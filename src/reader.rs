use std::{collections::HashMap, fs::File, io::Seek};

use anyhow::{anyhow, Context, Result};
use binrw::{binread, BinRead};
use serde_json::{json, Map, Value};

#[binread]
#[br(little, magic = b"\xA3\x95")]
struct PacketHeader {
    msg_id: u8,
}

#[binread]
#[br(little)] // FMT message
#[derive(Debug, Clone)]
pub struct FmtPacket {
    pub type_id: u8,
    length: u8,
    #[br(map = |bytes: [u8; 4]| sanitize_str(&bytes))]
    pub name: String,
    #[br(map = |bytes: [u8; 16]| sanitize_str(&bytes))]
    format_str: String,
    #[br(map = |bytes: [u8; 64]| sanitize_str(&bytes))]
    labels: String,
}

fn sanitize_str(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes)
        .trim_end_matches('\0')
        .to_string()
}

#[derive(Debug)]
enum LogValue {
    Int(i64),
    UInt(u64),
    Float(f32),
    Double(f64),
    Str(String),
}

impl std::fmt::Display for LogValue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LogValue::Int(v) => write!(f, "{}", v),
            LogValue::UInt(v) => write!(f, "{}", v),
            LogValue::Float(v) => write!(f, "{:4}", v),
            LogValue::Double(v) => write!(f, "{:6}", v),
            LogValue::Str(v) => write!(f, "\"{}\"", v),
        }
    }
}

impl From<LogValue> for Value {
    fn from(value: LogValue) -> Self {
        use LogValue::*;
        match value {
            Int(v) => json!(v),
            UInt(v) => json!(v),
            Float(v) => {
                if v.is_finite() {
                    json!(v)
                } else {
                    Value::Null
                }
            }
            Double(v) => {
                if v.is_finite() {
                    json!(v)
                } else {
                    Value::Null
                }
            }
            Str(v) => json!(v),
        }
    }
}

// we use u64 to be compatible with seek() and current_position() math.
fn field_length(fmt_char: char) -> Result<u64> {
    match fmt_char {
        'b' | 'B' | 'M' => Ok(1),
        'h' | 'c' | 'H' | 'C' => Ok(2),
        'i' | 'L' | 'I' | 'E' | 'e' | 'f' | 'n' => Ok(4),

        'q' | 'Q' | 'd' => Ok(8),
        'N' => Ok(16),
        'Z' => Ok(64),

        _ => Err(anyhow!("unexpcted char: {}", fmt_char)),
    }
}

fn parse_value(
    reader: &mut (impl std::io::Read + std::io::Seek),
    fmt_char: char,
) -> Result<LogValue> {
    match fmt_char {
        // signed ints
        'b' => Ok(LogValue::Int(i8::read_le(reader)? as i64)),
        'h' | 'c' => Ok(LogValue::Int(i16::read_le(reader)? as i64)),
        'i' | 'L' | 'e' => Ok(LogValue::Int(i32::read_le(reader)? as i64)),
        'q' => Ok(LogValue::Int(i64::read_le(reader)?)),

        // unsigned ints
        'B' | 'M' => Ok(LogValue::UInt(u8::read_le(reader)? as u64)),
        'H' | 'C' => Ok(LogValue::UInt(u16::read_le(reader)? as u64)),
        'I' | 'E' => Ok(LogValue::UInt(u32::read_le(reader)? as u64)),
        'Q' => Ok(LogValue::UInt(u64::read_le(reader)?)),

        // floats
        'f' => Ok(LogValue::Float(f32::read_le(reader)?)),
        'd' => Ok(LogValue::Double(f64::read_le(reader)?)),

        // Strings, fixed width
        'n' => {
            let mut buf = [0u8; 4];
            reader.read_exact(&mut buf)?;
            Ok(LogValue::Str(sanitize_str(&buf)))
        }
        'N' => {
            let mut buf = [0u8; 16];
            reader.read_exact(&mut buf)?;
            Ok(LogValue::Str(sanitize_str(&buf)))
        }
        'Z' => {
            let mut buf = [0u8; 64];
            reader.read_exact(&mut buf)?;
            Ok(LogValue::Str(sanitize_str(&buf)))
        }

        _ => Err(anyhow!("Unknown format char: {}", fmt_char)),
    }
}

pub struct ArduReader {
    filename: String,
    file: Option<File>,
    definitions: HashMap<u8, ArduDefinition>,
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
            file: None,
            definitions: HashMap::new(),
            last_timestamp: 0,
        }
    }

    pub fn read(&mut self) -> Result<ArduFrame> {
        if self.file.is_none() {
            self.file = Some(File::open(&self.filename).context("Failed opening file")?);
        }

        // we are now guaranteed unwrap will succeed.
        let file = self.file.as_mut().unwrap();

        let header = match PacketHeader::read(file) {
            Ok(h) => h,
            Err(binrw::Error::Io(e)) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                return Ok(ArduFrame::Eof)
            }
            Err(e) => {
                eprintln!("Unexpected error, but likely EOF: {}", e);
                return Ok(ArduFrame::Eof);
            }
        };

        if header.msg_id == 128 {
            let ardu_fmt = FmtPacket::read(file)?;

            let labels: Vec<String> = ardu_fmt
                .labels
                .split(",")
                .map(|s| s.trim().to_string())
                .collect();

            let definition = ArduDefinition {
                ardu_fmt: ardu_fmt.clone(),
                labels,
            };

            self.definitions
                .insert(ardu_fmt.type_id, definition.clone());

            return Ok(ArduFrame::ArduDefinition(definition));
        } else if let Some(definition) = self.definitions.get(&header.msg_id) {
            let mut current_ts = 0;
            let mut json_obj = Map::new();

            for (idx, c) in definition.ardu_fmt.format_str.chars().enumerate() {
                let val = parse_value(file, c);

                let label = definition.labels[idx].clone();

                let val = match val {
                    Ok(v) => v,
                    Err(e) => {
                        // if any of these fail, just let it fail with a "crpytic" error. Re-decorating the original error is too much trouble.
                        // Unless there's a cool syntax that allows it without too much boilerplate?

                        let file_size = file.metadata()?.len();
                        let current_pos = file.stream_position()?;
                        let field_len = field_length(c)?;

                        if current_pos + field_len > file_size {
                            // an incomplete file, which is ok.
                            eprintln!(
                                    "\nWARNING: file is incomplete, but read ok otherwise. Current position: {}. Expecting field of length: {}. File size: {}",
                                    current_pos, field_len, file_size
                                );
                            return Ok(ArduFrame::Eof);
                        }

                        // something happened that can't be "excused" by an unexpected EOF
                        return Err(e);
                    }
                };

                if label == "TimeUS" {
                    if let LogValue::UInt(v) = val {
                        current_ts = v * 1000;
                    }
                    if let LogValue::Int(v) = val {
                        current_ts = (v as u64) * 1000;
                    }
                }

                json_obj.insert(label.clone(), val.into());
            }

            if current_ts > 0 {
                self.last_timestamp = current_ts;
            } else {
                current_ts = self.last_timestamp;
            }

            let message = ArduMessage {
                type_id: header.msg_id,
                current_ts,
                json_obj,
            };

            return Ok(ArduFrame::ArduMessage(message));
        }

        Err(anyhow!(
            "Error: Unknown msg ID {} at position {}.",
            header.msg_id,
            file.stream_position()?
        ))
    }
}
