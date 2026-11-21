use anyhow::{anyhow, Context, Result};
use flightlog_core::{ColumnData, Log, ReadOptions};
use serde_json::{json, Map, Value};

#[derive(Debug, Clone)]
pub struct FmtPacket {
    pub type_id: u8,
    pub name: String,
}

pub struct ArduReader {
    filename: String,
    log: Option<Log>,
    next: usize,
    defined: Vec<bool>,
    last_timestamp: u64,
}

pub enum ArduFrame {
    ArduDefinition(ArduDefinition),
    ArduMessage(ArduMessage),
    Eof,
}

#[derive(Clone)]
pub struct ArduDefinition {
    pub ardu_fmt: FmtPacket,
    pub labels: Vec<String>,
}

pub struct ArduMessage {
    pub type_id: u8,
    pub current_ts: u64,
    pub json_obj: Map<String, Value>,
}

impl ArduReader {
    pub fn new(filename: &str) -> Self {
        Self {
            filename: filename.to_string(),
            log: None,
            next: 0,
            defined: Vec::new(),
            last_timestamp: 0,
        }
    }

    pub fn read(&mut self) -> Result<ArduFrame> {
        if self.log.is_none() {
            // flightlog-core resyncs past corrupt bytes and FMT-less ids and
            // decodes every format char ('a' included) instead of stopping.
            let opts = ReadOptions {
                scale: false,
                keep_order: true,
                ..ReadOptions::default()
            };
            let log = Log::open(&self.filename, &opts).context("Failed reading log")?;
            let integrity = &log.integrity;
            if integrity.skipped_bytes > 0 || !integrity.unknown_types.is_empty() {
                eprintln!(
                    "WARNING: skipped {} damaged bytes in {} regions, {} messages of unknown type",
                    integrity.skipped_bytes,
                    integrity.skipped_region_count,
                    integrity.unknown_types.values().sum::<u64>()
                );
            }
            self.defined = vec![false; log.tables().len()];
            self.log = Some(log);
        }

        // we are now guaranteed unwrap will succeed.
        let log = self.log.as_ref().unwrap();
        let order = log.order.as_deref().unwrap_or_default();

        // FMT rows are definitions, not messages; the old reader never emitted them.
        let (t, row, table) = loop {
            let Some(&(t, row)) = order.get(self.next) else {
                return Ok(ArduFrame::Eof);
            };
            let table = &log.tables()[t as usize];
            if table.name != "FMT" {
                break (t as usize, row as usize, table);
            }
            self.next += 1;
        };
        let type_id = u8::try_from(t).map_err(|_| anyhow!("More than 256 message types"))?;

        if !self.defined[t] {
            self.defined[t] = true;
            let labels: Vec<String> = table.column_names().iter().map(|s| s.to_string()).collect();
            let definition = ArduDefinition {
                ardu_fmt: FmtPacket {
                    type_id,
                    name: table.name.clone(),
                },
                labels,
            };
            return Ok(ArduFrame::ArduDefinition(definition));
        }
        self.next += 1;

        let mut current_ts = 0;
        let mut json_obj = Map::new();
        for col in &table.columns {
            if col.name == "TimeUS" {
                if let Some(v) = col.data.get_i64(row) {
                    current_ts = (v as u64) * 1000;
                }
            }
            json_obj.insert(col.name.clone(), cell(&col.data, row));
        }

        if current_ts > 0 {
            self.last_timestamp = current_ts;
        } else {
            current_ts = self.last_timestamp;
        }

        Ok(ArduFrame::ArduMessage(ArduMessage {
            type_id,
            current_ts,
            json_obj,
        }))
    }
}

fn cell(data: &ColumnData, row: usize) -> Value {
    match data {
        ColumnData::Float32(v) if v[row].is_finite() => json!(v[row]),
        ColumnData::Float64(v) if v[row].is_finite() => json!(v[row]),
        ColumnData::Float32(_) | ColumnData::Float64(_) => Value::Null,
        ColumnData::UInt64(v) => json!(v[row]),
        ColumnData::Utf8(v) | ColumnData::Binary(v) => {
            json!(String::from_utf8_lossy(v.get(row)))
        }
        ColumnData::FixedList { size, values } => Value::Array(
            (row * size..row * size + size)
                .map(|i| cell(values, i))
                .collect(),
        ),
        other => json!(other.get_i64(row)),
    }
}
