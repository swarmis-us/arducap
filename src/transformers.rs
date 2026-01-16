use crate::reader::{ArduDefinition, ArduMessage, FmtPacket};
use anyhow::Result;
use serde_json::{json, Map};
use std::collections::HashMap;

fn generate_json_schema(fmt: &FmtPacket, labels: &[String]) -> String {
    let mut props = Map::new();

    for label in labels {
        props.insert(label.clone(), json!({"type": "number"}));
    }

    let schema_json = json!({
        "type": "object",
        "title": fmt.name,
        "properties": props
    });

    serde_json::to_string(&schema_json).unwrap()
}

pub struct TransformedMessage {
    pub topic: String,
    pub schema_name: String,
    pub schema_encoding: String,
    pub schema_data: Vec<u8>,
    pub payload: Vec<u8>,
}

pub trait Transformer {
    fn check_registered_to_transform(&mut self, definition: &ArduDefinition) -> bool;

    fn transform(&mut self, msg: &ArduMessage) -> Result<Vec<TransformedMessage>>;
}

pub struct GenericTransformer {
    schemas: HashMap<u8, (String, Vec<u8>)>,
}

impl GenericTransformer {
    pub fn new() -> Self {
        Self {
            schemas: HashMap::new(),
        }
    }
}

impl Transformer for GenericTransformer {
    fn check_registered_to_transform(&mut self, definition: &ArduDefinition) -> bool {
        let schema_str = generate_json_schema(&definition.ardu_fmt, &definition.labels);
        self.schemas.insert(
            definition.ardu_fmt.type_id,
            (definition.ardu_fmt.name.to_owned(), schema_str.into_bytes()),
        );

        true
    }

    fn transform(&mut self, msg: &ArduMessage) -> Result<Vec<TransformedMessage>> {
        let (name, schema_bytes) = self.schemas.get(&msg.type_id).unwrap();

        Ok(vec![TransformedMessage {
            topic: format!("/ardupilot/{}", name),
            schema_name: name.clone(),
            schema_encoding: "jsonschema".to_string(),
            schema_data: schema_bytes.clone(),
            payload: serde_json::to_vec(&msg.json_obj)?,
        }])
    }
}

const LOCATION_FIX_SCHEMA: &str = r#"{
  "type": "object",
  "properties": {
    "latitude": { "type": "number" },
    "longitude": { "type": "number" },
    "altitude": { "type": "number" },
    "position_covariance_type": { "type": "integer" },
    "position_covariance": { "type": "array", "items": { "type": "number" } }
  }
}"#;

const FRAME_TRANSFORM_SCHEMA: &str = r#"{
  "type": "object",
  "properties": {
    "timestamp": {
      "type": "object",
      "properties": { "sec": { "type": "integer" }, "nsec": { "type": "integer" } }
    },
    "parent_frame_id": { "type": "string" },
    "child_frame_id": { "type": "string" },
    "translation": {
      "type": "object",
      "properties": { "x": {"type":"number"}, "y": {"type":"number"}, "z": {"type":"number"} }
    },
    "rotation": {
      "type": "object",
      "properties": { "x": {"type":"number"}, "y": {"type":"number"}, "z": {"type":"number"}, "w": {"type":"number"} }
    }
  }
}"#;

pub struct FoxgloveFusedTransformer {
    home: Option<(f64, f64, f64)>, // Lat, Lon, Alt
    current_pos: (f64, f64, f64),  // Lat, Lon, Alt
    current_att: (f64, f64, f64),  // Roll, Pitch, Yaw (centi-degrees)
    has_seen_pos: bool,
    topic_map: HashMap<u8, String>,
}

impl FoxgloveFusedTransformer {
    pub fn new() -> Self {
        Self {
            home: None,
            current_pos: (0.0, 0.0, 0.0),
            current_att: (0.0, 0.0, 0.0),
            has_seen_pos: false,
            topic_map: HashMap::new(),
        }
    }
}

fn euler_to_quat(roll_cd: f64, pitch_cd: f64, yaw_cd: f64) -> (f64, f64, f64, f64) {
    // 1. Convert Centi-degrees to Radians
    let r = (roll_cd / 100.0).to_radians();
    let p = (pitch_cd / 100.0).to_radians();
    let y = (yaw_cd / 100.0).to_radians();

    // 2. Calculate native NED Quaternion (Standard Aerospace Sequence: Z-Y-X)
    let cy = (y * 0.5).cos();
    let sy = (y * 0.5).sin();
    let cp = (p * 0.5).cos();
    let sp = (p * 0.5).sin();
    let cr = (r * 0.5).cos();
    let sr = (r * 0.5).sin();

    // These are the components of the rotation in the NED frame
    let q_w = cr * cp * cy + sr * sp * sy;
    let q_x = sr * cp * cy - cr * sp * sy;
    let q_y = cr * sp * cy + sr * cp * sy;
    let q_z = cr * cp * sy - sr * sp * cy;

    // 3. Convert NED to ENU (Foxglove)
    // To rotate the frame 180° around X (Forward):
    // X stays X, Y becomes -Y, Z becomes -Z
    // The quaternion conjugate for this transformation is (x, -y, -z, w)

    // NOTE: If your map heading is off by 90 degrees (East vs North),
    // you might need to adjust Yaw before step 2, but try this first.
    (q_x, -q_y, -q_z, q_w)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_euler_to_quat_ned_to_enu() {
        // Case 1: Identity (Level flight, facing North)
        // ArduPilot (NED): Roll=0, Pitch=0, Yaw=0
        // Foxglove (ENU):  Should be level, facing North (which is +Y in standard ENU, or +X depending on viewer)
        // Let's check the raw quaternion output.
        // NED Identity Quat: (0, 0, 0, 1) [x, y, z, w]
        // ENU Conversion (swap y, z signs): (0, -0, -0, 1) -> (0, 0, 0, 1)
        let (x, y, z, w) = euler_to_quat(0.0, 0.0, 0.0);

        assert_relative_eq!(x, 0.0);
        assert_relative_eq!(y, 0.0);
        assert_relative_eq!(z, 0.0);
        assert_relative_eq!(w, 1.0);

        // Case 2: 90 Degree Yaw (Facing East)
        // ArduPilot Yaw = 9000 centi-degrees
        let (x, y, z, w) = euler_to_quat(0.0, 0.0, 9000.0);

        // In NED, 90 deg yaw around Z = 0.707 + 0.707k (w=0.707, z=0.707)
        // Our converter swaps Z sign -> w=0.707, z=-0.707
        // This effectively mirrors the rotation, which maps "Right" (NED) to "Left" (ENU) correctly?

        let diag_trig = 2.0f64.sqrt() / 2.0;
        assert_relative_eq!(x, 0.0);
        assert_relative_eq!(y, 0.0);
        assert_relative_eq!(z, -diag_trig);
        assert_relative_eq!(w, diag_trig);

        // for those who don't believe Pythagoras, nevermind simple algebra
        assert_relative_eq!(x * x + y * y + z * z + w * w, 1.0);
    }
}

// We must account for earth curvature in our ENU calculations
// Conversions to ECEF are necessary. See more here: https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
// https://en.wikipedia.org/wiki/World_Geodetic_System#WGS_84
// We include the math implementation here, to minimize the external dependencies.

// WGS-84 Ellipsoid Constants
const WGS84_A: f64 = 6378137.0;
const WGS84_F: f64 = 1.0 / 298.257223563;
const WGS84_E2: f64 = WGS84_F * (2.0 - WGS84_F);

fn wgs84_to_enu(
    lat: f64,
    lon: f64,
    alt: f64,
    home_lat: f64,
    home_lon: f64,
    home_alt: f64,
) -> (f64, f64, f64) {
    // 1. LLA to ECEF (Earth-Centered)
    let to_ecef = |lat_d: f64, lon_d: f64, alt_m: f64| -> (f64, f64, f64) {
        let lat_rad = lat_d.to_radians();
        let lon_rad = lon_d.to_radians();
        let n = WGS84_A / (1.0 - WGS84_E2 * lat_rad.sin().powi(2)).sqrt();
        (
            (n + alt_m) * lat_rad.cos() * lon_rad.cos(),
            (n + alt_m) * lat_rad.cos() * lon_rad.sin(),
            (n * (1.0 - WGS84_E2) + alt_m) * lat_rad.sin(),
        )
    };

    let (hx, hy, hz) = to_ecef(home_lat, home_lon, home_alt);
    let (px, py, pz) = to_ecef(lat, lon, alt);

    // 2. ECEF Vector to ENU Frame
    let dx = px - hx;
    let dy = py - hy;
    let dz = pz - hz;

    let h_lat_rad = home_lat.to_radians();
    let h_lon_rad = home_lon.to_radians();
    let sin_lat = h_lat_rad.sin();
    let cos_lat = h_lat_rad.cos();
    let sin_lon = h_lon_rad.sin();
    let cos_lon = h_lon_rad.cos();

    (
        -sin_lon * dx + cos_lon * dy,                                    // East
        -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz, // North
        cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz,  // Up
    )
}

const GPS: &str = "GPS";
const ATT: &str = "ATT";
const POS: &str = "POS";

impl Transformer for FoxgloveFusedTransformer {
    fn check_registered_to_transform(&mut self, definition: &ArduDefinition) -> bool {
        let n = &definition.ardu_fmt.name;

        if [GPS, ATT, POS].contains(&n.as_str()) {
            self.topic_map
                .insert(definition.ardu_fmt.type_id, n.clone());
            true
        } else {
            false
        }
    }

    fn transform(&mut self, msg: &ArduMessage) -> Result<Vec<TransformedMessage>> {
        let mut output = Vec::new();
        let json = &msg.json_obj;

        // this unwrap should never fail, unless there's a critical bug in the caller pipeline.
        let topic_name = self.topic_map.get(&msg.type_id).unwrap();

        if topic_name == GPS && self.has_seen_pos {
            return Ok(vec![]);
        }

        if topic_name == POS {
            self.has_seen_pos = true;
        }

        // 1. Ingest Data
        let has_position = topic_name == GPS || topic_name == POS;
        let has_att = topic_name == ATT;

        if has_position {
            let get_int = |k| json.get(k).and_then(|v| v.as_i64());
            let get_flt = |k| json.get(k).and_then(|v| v.as_f64());

            let lat = get_int("Lat").or(get_int("Latitude")).unwrap_or(0) as f64 / 1.0e7;
            let lon = get_int("Lng").or(get_int("Longitude")).unwrap_or(0) as f64 / 1.0e7;

            // GPS altitude data is in centimeters, we need to convet. POS data is in meters, which is fine.
            let altitude_scale_factor = if topic_name == GPS { 0.01 } else { 1.0 };
            let alt = get_flt("Alt").or(get_flt("Altitude")).unwrap_or(0.0) * altitude_scale_factor;

            // Set Home ONLY ONCE
            if self.home.is_none() && lat.abs() > 0.1 {
                self.home = Some((lat, lon, alt));

                // EMIT ANCHOR: Tells 3D panel "world" frame is at this Lat/Lon
                let anchor_obj = json!({
                    "frame_id": "world", // This pins the 'world' frame to the map
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": alt
                });
                output.push(TransformedMessage {
                    topic: "/foxglove/map_origin".to_string(),
                    schema_name: "foxglove.LocationFix".to_string(),
                    schema_encoding: "jsonschema".to_string(),
                    schema_data: LOCATION_FIX_SCHEMA.as_bytes().to_vec(),
                    payload: serde_json::to_vec(&anchor_obj)?,
                });
            }
            self.current_pos = (lat, lon, alt);

            // EMIT TRACE: For the 2D Map Panel
            let trace_obj = json!({
                "frame_id": "base_link",
                "latitude": lat,
                "longitude": lon,
                "altitude": alt
            });
            output.push(TransformedMessage {
                topic: "/foxglove/gps".to_string(), // 2D Panel listens to this
                schema_name: "foxglove.LocationFix".to_string(),
                schema_encoding: "jsonschema".to_string(),
                schema_data: LOCATION_FIX_SCHEMA.as_bytes().to_vec(),
                payload: serde_json::to_vec(&trace_obj)?,
            });
        }

        if has_att {
            let get_flt = |k| json.get(k).and_then(|v| v.as_f64()).unwrap_or(0.0);
            self.current_att = (get_flt("Roll"), get_flt("Pitch"), get_flt("Yaw"));
        }

        // 2. Emit 3D Transform (Only if we have a home)
        if let Some((home_lat, home_lon, home_alt)) = self.home {
            let (e, n, u) = wgs84_to_enu(
                self.current_pos.0,
                self.current_pos.1,
                self.current_pos.2,
                home_lat,
                home_lon,
                home_alt,
            );

            // Convert to Quaternion
            let (qx, qy, qz, qw) =
                euler_to_quat(self.current_att.0, self.current_att.1, self.current_att.2);

            let tf_obj = json!({
                "timestamp": { "sec": msg.current_ts / 1_000_000_000, "nsec": msg.current_ts % 1_000_000_000 },
                "parent_frame_id": "world",
                "child_frame_id": "base_link",
                "translation": { "x": e, "y": n, "z": u }, // ENU: East=X, North=Y, Up=Z
                "rotation": { "x": qx, "y": qy, "z": qz, "w": qw }
            });

            output.push(TransformedMessage {
                topic: "/foxglove/base_link_transform".to_string(),
                schema_name: "foxglove.FrameTransform".to_string(),
                schema_encoding: "jsonschema".to_string(),
                schema_data: FRAME_TRANSFORM_SCHEMA.as_bytes().to_vec(),
                payload: serde_json::to_vec(&tf_obj)?,
            });
        }

        Ok(output)
    }
}
