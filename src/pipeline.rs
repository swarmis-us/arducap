use std::{
    collections::{BTreeMap, HashMap},
    fs::File,
    path::{Path, PathBuf},
};

use anyhow::Result;

use mcap::{records::MessageHeader, Writer};

use crate::{
    reader::{ArduFrame, ArduReader},
    transformers::{FoxgloveFusedTransformer, GenericTransformer, Transformer},
};

fn with_mcap_extension(name: &str) -> PathBuf {
    let mut p = Path::new(name).to_path_buf();
    p.set_extension("mcap");
    p
}

struct McapChannelInfo {
    channel_id: u16,
    sequence: u32,
}

pub fn process_ardupilot_file(filename: &str) -> Result<()> {
    let mut reader = ArduReader::new(filename);
    let mcap_filename = with_mcap_extension(filename);

    let mcap_file = File::create(mcap_filename)?;
    let mut mcap_writer = Writer::new(mcap_file)?;

    let mut channel_map = HashMap::<(String, String), McapChannelInfo>::new();

    let mut transformers: Vec<Box<dyn Transformer>> = vec![
        Box::new(GenericTransformer::new()),
        Box::new(FoxgloveFusedTransformer::new()),
    ];

    let mut subscriptions = HashMap::<u8, Vec<usize>>::new();

    loop {
        match reader.read()? {
            ArduFrame::Eof => {
                mcap_writer.finish()?;
                return Ok(());
            }
            ArduFrame::ArduDefinition(definition) => {
                let mut active_indices = Vec::new();
                for (i, t) in transformers.iter_mut().enumerate() {
                    if t.check_registered_to_transform(&definition) {
                        active_indices.push(i);
                    }
                }

                subscriptions.insert(definition.ardu_fmt.type_id, active_indices);
            }
            ArduFrame::ArduMessage(message) => {
                if let Some(indices) = subscriptions.get(&message.type_id) {
                    for &i in indices {
                        let out_msgs = transformers[i].transform(&message)?;

                        for out_msg in out_msgs {
                            let key = (out_msg.topic.clone(), out_msg.schema_name.clone());

                            if !channel_map.contains_key(&key) {
                                let schema_id = mcap_writer.add_schema(
                                    &out_msg.schema_name,
                                    &out_msg.schema_encoding,
                                    &out_msg.schema_data,
                                )?;

                                let channel_id = mcap_writer.add_channel(
                                    schema_id,
                                    &out_msg.topic,
                                    "json",
                                    &BTreeMap::new(),
                                )?;

                                channel_map.insert(
                                    key.clone(),
                                    McapChannelInfo {
                                        channel_id,
                                        sequence: 0,
                                    },
                                );
                            }

                            let channel_info = channel_map.get_mut(&key).unwrap();
                            mcap_writer.write_to_known_channel(
                                &MessageHeader {
                                    channel_id: channel_info.channel_id,
                                    sequence: channel_info.sequence,
                                    log_time: message.current_ts,
                                    publish_time: message.current_ts,
                                },
                                &out_msg.payload,
                            )?;

                            channel_info.sequence += 1;
                        }
                    }
                }
            }
        }
    }
}
