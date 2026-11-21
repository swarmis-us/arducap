use arducap::reader::{ArduFrame, ArduReader};

fn fmt(type_id: u8, len: u8, name: &str, format: &str, labels: &str) -> Vec<u8> {
    let mut b = vec![0xA3, 0x95, 0x80, type_id, len];
    for (s, n) in [(name, 4), (format, 16), (labels, 64)] {
        let mut f = s.as_bytes().to_vec();
        f.resize(n, 0);
        b.extend(f);
    }
    b
}

fn tst(time_us: u64, v: i32) -> Vec<u8> {
    let mut b = vec![0xA3, 0x95, 200];
    b.extend(time_us.to_le_bytes());
    b.extend(v.to_le_bytes());
    b
}

#[test]
fn reads_past_corruption_unknown_ids_and_arrays() {
    let mut log = fmt(0x80, 89, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns");
    log.extend(fmt(200, 15, "TST", "Qi", "TimeUS,V"));
    log.extend(fmt(201, 75, "ISBD", "Qa", "TimeUS,X"));
    log.extend(tst(1_000, 1));
    log.extend([0xA3, 0x00, 0x42]); // damaged header
    log.extend(tst(2_000, 2));
    log.extend([0xA3, 0x95, 99, 1, 2, 3]); // id with no FMT
    log.extend(tst(3_000, 3));
    let mut isbd = vec![0xA3, 0x95, 201];
    isbd.extend(4_000u64.to_le_bytes());
    for i in 0..32i16 {
        isbd.extend((-i).to_le_bytes());
    }
    log.extend(isbd);
    log.extend(tst(5_000, 5));

    let path = std::env::temp_dir().join("arducap_reader_test.bin");
    std::fs::write(&path, &log).unwrap();

    let mut reader = ArduReader::new(path.to_str().unwrap());
    let mut values = Vec::new();
    let mut array = None;
    loop {
        match reader.read().unwrap() {
            ArduFrame::Eof => break,
            ArduFrame::ArduDefinition(d) => assert_ne!(d.ardu_fmt.name, "FMT"),
            ArduFrame::ArduMessage(m) => {
                assert!(!m.json_obj.contains_key("Format"));
                if let Some(v) = m.json_obj.get("V") {
                    values.push((m.current_ts, v.as_i64().unwrap()));
                }
                if let Some(x) = m.json_obj.get("X") {
                    array = Some(x.as_array().unwrap().len());
                }
            }
        }
    }
    std::fs::remove_file(&path).ok();

    assert_eq!(
        values,
        vec![
            (1_000_000, 1),
            (2_000_000, 2),
            (3_000_000, 3),
            (5_000_000, 5)
        ]
    );
    assert_eq!(array, Some(32));
}
