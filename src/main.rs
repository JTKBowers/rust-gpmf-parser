use core::fmt::Debug;
use std::io;
use std::io::prelude::*;
use std::fs::File;


use nom::error::ErrorKind;
use nom::bytes::streaming::{tag, take};
use nom::number::streaming::{be_u8, be_u16, be_u32, be_u64, be_i8, be_i16, be_i32, be_f32};
use nom::IResult;

#[derive(Debug)]
enum ParseError {
    Generic,
    ReadError
}

impl<T: Debug> From<nom::Err<T>> for ParseError {
    fn from(err: nom::Err<T>) -> Self {
        println!("{:?}", err);
        ParseError::Generic
    }
}

impl From<std::io::Error> for ParseError {
    fn from(_err: std::io::Error) -> Self {
        ParseError::ReadError
    }
}

#[derive(Debug)]
enum Block {
    DeviceSource(Vec<Block>),
    DeviceID([u8; 4]),
    DeviceName(String),
    Stream(Vec<Block>),
    StartTimestamp(u64),
    TotalSamples(u32),
    StreamName(String),
    InputOrientation(String),
    UnitsSI(String),
    ScalingFactorS(i16),
    ScalingFactorL(Vec<i32>), // For GPS. Should tidy this up.
    Temperature(f32),
    Acceleration(Vec<[i16; 3]>),
    Gyroscope(Vec<[i16; 3]>),
    ShutterSpeed(Vec<f32>),
    WhiteBalance(Vec<u16>),
    WhiteBalanceRGBGains(Vec<[f32; 3]>),
    ISO(Vec<u16>),
    ImageUniformity(Vec<f32>),
    Type(String),
    Custom(String, Vec<u8>),
    GPSF(u32),
    GPSTimestamp(String),
    GPSP(u16), // precision?
    GPSA([u8; 4]),
    GPS5(Vec<u8>),
    CameraOrientation(Vec<[i16; 4]>),
    ImageOrientation(Vec<[i16; 4]>),
    GravityVector(Vec<[i16; 3]>),
    WindProcessing(Vec<(u8, u8)>),
    MicrophoneWet(Vec<(u8, u8, u8)>),
    AGCAudioLevel(Vec<(i8, i8)>),
    MRVFrameSkip(Vec<i16>),
    LRVO(i8),
    LRVS(i8),
    LRVFrameSkip(Vec<i16>),
}

fn parse_devc(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(&[0x0])(input)?;

    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let (input, block_bytes) = take(size as usize * count as usize)(input)?;

    let (trailing_bytes, sub_blocks) = parser(block_bytes)?;
    assert_eq!(trailing_bytes.len(), 0);

    Ok((input, Block::DeviceSource(sub_blocks)))
}

fn parse_dvid(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"L")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 4);
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let (input, device_id) = take((size as usize)*(count as usize))(input)?;

    let mut device_id_array = [0u8; 4];
    device_id_array.copy_from_slice(device_id);

    Ok((input, Block::DeviceID(device_id_array)))
}

fn parse_dvnm(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"c")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let string_length = (size as usize)*(count as usize);
    let (input, device_name) = take(string_length)(input)?;
    let device_name = std::str::from_utf8(device_name).unwrap();

    // Take remaining padding bytes
    let (input, _padding) = if string_length % 4 != 0 {
        let count_remaining_bytes = 4 - string_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::DeviceName(device_name.to_string())))
}

fn parse_strm(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(&[0x0])(input)?;

    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let (input, block_bytes) = take(size as usize * count as usize)(input)?;

    let (trailing_bytes, sub_blocks) = parser(block_bytes)?;
    assert_eq!(trailing_bytes.len(), 0);

    Ok((input, Block::Stream(sub_blocks)))
}

fn parse_stmp(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"J")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 8);
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let (input, start_timestamp) = be_u64(input)?;

    Ok((input, Block::StartTimestamp(start_timestamp)))
}


fn parse_tsmp(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"L")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 4);
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let (input, total_samples) = be_u32(input)?;

    Ok((input, Block::TotalSamples(total_samples)))
}


fn parse_stnm(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"c")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let string_length = (size as usize)*(count as usize);
    let (input, stream_name) = take(string_length)(input)?;
    let stream_name = std::str::from_utf8(stream_name).unwrap();

    // Take remaining padding bytes
    let (input, _padding) = if string_length % 4 != 0 {
        let count_remaining_bytes = 4 - string_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::StreamName(stream_name.to_string())))
}

fn parse_orin(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"c")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let string_length = (size as usize)*(count as usize);
    let (input, orientation) = take(string_length)(input)?;
    let orientation = std::str::from_utf8(orientation).unwrap();

    // Take remaining padding bytes
    let (input, _padding) = if string_length % 4 != 0 {
        let count_remaining_bytes = 4 - string_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::InputOrientation(orientation.to_string())))
}

fn parse_siun(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"c")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let string_length = (size as usize)*(count as usize);
    let (input, si_units) = take(string_length)(input)?;

    let mut si_units = si_units.to_vec();
    for i in 0..si_units.len() {
        if si_units[i] == 0xb2 { // Seems to represent ^-2
            si_units[i] = b"2"[0];
        }
    }
    let si_units = std::str::from_utf8(&si_units).unwrap();

    // Take remaining padding bytes
    let (input, _padding) = if string_length % 4 != 0 {
        let count_remaining_bytes = 4 - string_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::UnitsSI(si_units.to_string())))
}

fn parse_scal(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, data_type) = take(1usize)(input)?;

    if data_type == b"s" {
        let (input, size) = be_u8(input)?;
        assert_eq!(size, 2);
        let (input, count) = be_u16(input)?;
        assert_eq!(count, 1);

        let (input, scaling_factor) = be_i16(input)?;

        let (input, _) = take(2usize)(input)?;

        Ok((input, Block::ScalingFactorS(scaling_factor)))
    } else if data_type == b"l" {
        let (input, size) = be_u8(input)?;
        assert_eq!(size, 4);
        let (input, count) = be_u16(input)?;

        let mut input = input;
        let mut scaling_factors = Vec::new();
        for _ in 0..count {
            let (iinput, scaling_factor) = be_i32(input)?;
            scaling_factors.push(scaling_factor);
            input = iinput; // TODO: tidy up
        }

        Ok((input, Block::ScalingFactorL(scaling_factors)))
    } else {
        panic!("Unexpected scaling factor data type {}", std::str::from_utf8(data_type).unwrap());
    }
}

fn parse_tmpc(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"f")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 4);
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let (input, temperature_celsius) = be_f32(input)?;

    Ok((input, Block::Temperature(temperature_celsius)))
}

fn parse_accl(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"s")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 6); // Each measurement is a triplet
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, d1) = be_i16(input)?;
        let (iinput, d2) = be_i16(iinput)?;
        let (iinput, d3) = be_i16(iinput)?;
        measurements.push([d1, d2, d3]);
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = 6 * measurements.len();
    let input = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?.0
    } else {
        input
    };

    Ok((input, Block::Acceleration(measurements)))
}

fn parse_gyro(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"s")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 6); // Each measurement is a triplet
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, d1) = be_i16(input)?;
        let (iinput, d2) = be_i16(iinput)?;
        let (iinput, d3) = be_i16(iinput)?;
        measurements.push([d1, d2, d3]);
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = 6 * measurements.len();
    let input = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?.0
    } else {
        input
    };

    Ok((input, Block::Gyroscope(measurements)))
}

fn parse_shut(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"f")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 4);
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, shutter_speed) = be_f32(input)?;
        measurements.push(shutter_speed);
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = 6 * measurements.len();
    let input = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?.0
    } else {
        input
    };

    Ok((input, Block::ShutterSpeed(measurements)))
}

fn parse_wbal(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"S")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 2);
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, white_balance) = be_u16(input)?;
        measurements.push(white_balance);
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = 6 * measurements.len();
    let input = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?.0
    } else {
        input
    };

    Ok((input, Block::WhiteBalance(measurements)))
}

fn parse_wrgb(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"f")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 12);
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, d1) = be_f32(input)?;
        let (iinput, d2) = be_f32(iinput)?;
        let (iinput, d3) = be_f32(iinput)?;
        measurements.push([d1, d2, d3]);
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = 12 * measurements.len();
    let input = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?.0
    } else {
        input
    };

    Ok((input, Block::WhiteBalanceRGBGains(measurements)))
}

fn parse_isoe(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"S")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 2);
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, iso) = be_u16(input)?;
        measurements.push(iso);
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = 6 * measurements.len();
    let input = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?.0
    } else {
        input
    };

    Ok((input, Block::ISO(measurements)))
}

fn parse_unif(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"f")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 4);
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, shutter_speed) = be_f32(input)?;
        measurements.push(shutter_speed);
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = 6 * measurements.len();
    let input = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?.0
    } else {
        input
    };

    Ok((input, Block::ImageUniformity(measurements)))
}

fn parse_type(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"c")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let string_length = (size as usize)*(count as usize);
    let (input, stream_name) = take(string_length)(input)?;
    let stream_name = std::str::from_utf8(stream_name).unwrap();

    // Take remaining padding bytes
    let (input, _padding) = if string_length % 4 != 0 {
        let count_remaining_bytes = 4 - string_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::Type(stream_name.to_string())))
}

fn parse_custom<'a>(type_name: &'a[u8], input: &'a[u8]) -> IResult<&'a[u8], Block> {
    let type_name = std::str::from_utf8(type_name).unwrap();

    let (input, _data_type) = tag(b"?")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let data_length = (size as usize)*(count as usize);
    let (input, data_bytes) = take(data_length)(input)?;
    let input = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?.0
    } else {
        input
    };

    Ok((input, Block::Custom(type_name.to_string(), data_bytes.to_vec())))
}

fn parse_gpsf(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"L")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 4);
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let (input, gpsf) = be_u32(input)?;

    Ok((input, Block::GPSF(gpsf)))
}

fn parse_gpsu(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"U")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let string_length = (size as usize)*(count as usize);
    let (input, gps_timestamp) = take(string_length)(input)?;
    let gps_timestamp = std::str::from_utf8(gps_timestamp).unwrap();

    // Take remaining padding bytes
    let (input, _padding) = if string_length % 4 != 0 {
        let count_remaining_bytes = 4 - string_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::GPSTimestamp(gps_timestamp.to_string())))
}

fn parse_gpsp(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"S")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 2);
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let (input, unknown) = be_u16(input)?;
    let (input, _) = tag(&[0,0])(input)?;

    Ok((input, Block::GPSP(unknown)))
}

fn parse_gpsa(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"F")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 4);
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let (input, key) = take(4usize)(input)?;

    let mut key_array = [0u8; 4];
    key_array.copy_from_slice(key);

    Ok((input, Block::GPSA(key_array)))
}

fn parse_gps5(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"l")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let count_bytes = (size as usize)*(count as usize);
    let (input, payload) = take(count_bytes)(input)?;

    Ok((input, Block::GPS5(payload.to_vec())))
}

fn parse_cori(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"s")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 8); // Each measurement is a quartet
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, d1) = be_i16(input)?;
        let (iinput, d2) = be_i16(iinput)?;
        let (iinput, d3) = be_i16(iinput)?;
        let (iinput, d4) = be_i16(iinput)?;
        measurements.push([d1, d2, d3, d4]);
        input = iinput; // TODO: tidy up
    }

    Ok((input, Block::CameraOrientation(measurements)))
}

fn parse_iori(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"s")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 8); // Each measurement is a quartet
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, d1) = be_i16(input)?;
        let (iinput, d2) = be_i16(iinput)?;
        let (iinput, d3) = be_i16(iinput)?;
        let (iinput, d4) = be_i16(iinput)?;
        measurements.push([d1, d2, d3, d4]);
        input = iinput; // TODO: tidy up
    }

    Ok((input, Block::ImageOrientation(measurements)))
}

fn parse_grav(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"s")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 6); // Each measurement is a triplet
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, d1) = be_i16(input)?;
        let (iinput, d2) = be_i16(iinput)?;
        let (iinput, d3) = be_i16(iinput)?;
        measurements.push([d1, d2, d3]);
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = 6 * measurements.len();
    let input = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?.0
    } else {
        input
    };

    Ok((input, Block::GravityVector(measurements)))
}

fn parse_wndm(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"B")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 2);
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, enable) = be_u8(input)?;
        let (iinput, meter_value) = be_u8(iinput)?;
        measurements.push((enable, meter_value));
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = size as usize*count as usize;
    let (input, _padding) = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::WindProcessing(measurements)))
}

fn parse_mwet(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"B")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 3);
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, mic_wet) = be_u8(input)?;
        let (iinput, all_mics) = be_u8(iinput)?;
        let (iinput, confidence) = be_u8(iinput)?;
        measurements.push((mic_wet, all_mics, confidence));
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = size as usize*count as usize;
    let (input, _padding) = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::MicrophoneWet(measurements)))
}

fn parse_aalp(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"b")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 2);
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, rms_level) = be_i8(input)?;
        let (iinput, peak_level) = be_i8(iinput)?;
        measurements.push((rms_level, peak_level));
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = size as usize*count as usize;
    let (input, _padding) = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::AGCAudioLevel(measurements)))
}

fn parse_mskp(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"s")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 2);
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, unknown) = be_i16(input)?;
        measurements.push(unknown);
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = size as usize*count as usize;
    let (input, _padding) = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::MRVFrameSkip(measurements)))
}


fn parse_lrvo(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"b")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 1);
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let (input, value) = be_i8(input)?;
    let (input, _) = take(3usize)(input)?;

    Ok((input, Block::LRVO(value)))
}


fn parse_lrvs(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"b")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 1);
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let (input, value) = be_i8(input)?;
    let (input, _) = take(3usize)(input)?;

    Ok((input, Block::LRVS(value)))
}

fn parse_lskp(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"s")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 2);
    let (input, count) = be_u16(input)?;

    let mut input = input;
    let mut measurements = Vec::new();
    for _ in 0..count {
        let (iinput, unknown) = be_i16(input)?;
        measurements.push(unknown);
        input = iinput; // TODO: tidy up
    }

    // Take remaining padding bytes
    let data_length = size as usize*count as usize;
    let (input, _padding) = if data_length % 4 != 0 {
        let count_remaining_bytes = 4 - data_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::LRVFrameSkip(measurements)))
}

fn parse_block(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, block_type) = take(4usize)(input)?;
    let (input, block) = match block_type {
        b"DEVC" => parse_devc(input),
        b"DVID" => parse_dvid(input),
        b"DVNM" => parse_dvnm(input),
        b"STRM" => parse_strm(input),
        b"STMP" => parse_stmp(input),
        b"TSMP" => parse_tsmp(input),
        b"STNM" => parse_stnm(input),
        b"ORIN" => parse_orin(input),
        b"SIUN" => parse_siun(input),
        b"UNIT" => parse_siun(input),
        b"SCAL" => parse_scal(input),
        b"TMPC" => parse_tmpc(input),
        b"ACCL" => parse_accl(input),
        b"GYRO" => parse_gyro(input),
        b"SHUT" => parse_shut(input),
        b"WBAL" => parse_wbal(input),
        b"WRGB" => parse_wrgb(input),
        b"ISOE" => parse_isoe(input),
        b"UNIF" => parse_unif(input),
        b"TYPE" => parse_type(input),
        b"GPSF" => parse_gpsf(input),
        b"GPSU" => parse_gpsu(input),
        b"GPSP" => parse_gpsp(input),
        b"GPSA" => parse_gpsa(input),
        b"GPS5" => parse_gps5(input),
        b"CORI" => parse_cori(input),
        b"IORI" => parse_iori(input),
        b"GRAV" => parse_grav(input),
        b"WNDM" => parse_wndm(input),
        b"MWET" => parse_mwet(input),
        b"AALP" => parse_aalp(input),
        b"MSKP" => parse_mskp(input),
        b"LRVO" => parse_lrvo(input),
        b"LRVS" => parse_lrvs(input),
        b"LSKP" => parse_lskp(input),
        block_type => {
            let r = parse_custom(block_type, input);
            if r.is_err() {
                println!("Got unexpected block type {:x?} | {:?}", block_type, std::str::from_utf8(block_type).unwrap());
                Err(nom::Err::Failure(nom::error::Error::new(input, ErrorKind::Tag)))
            } else {
                r
            }
        }
    }?;

    Ok((input, block))
}

fn parser(input: &[u8]) -> IResult<&[u8], Vec<Block>> {
    let mut blocks = Vec::new();
    let mut input = input;
    loop {
        let result = parse_block(input)?;
        input = result.0;
        blocks.push(result.1);

        if input == b"" {
            break
        }
    }
    Ok((input, blocks))
}

// TODO: streaming
// fn parse_metadata<T: Read>(mut f: T) -> Result<Vec<u8>, ParseError> {
//     let mut buffer = [0; 10];
//     let bytes_read = f.read(&mut buffer)?;

//     let mut input = &buffer[..bytes_read];
//     loop {
//         let result = parser(input);

//         if let Ok((new_input, _x)) = result {
//             input = new_input;
//         }

//         if let Err(nom::Err::Incomplete(_)) = result {
//             let bytes_read = f.read(&mut buffer)?;
//             input = &buffer[..bytes_read];
//             continue;
//         }

//         result?;
//     }

//     // Ok(vec![])
// }

fn parse_metadata<T: Read>(mut f: T) -> Result<Vec<Block>, ParseError> {
    let mut buffer = Vec::new();
    let bytes_read = f.read_to_end(&mut buffer)?;

    let input = &buffer[..bytes_read];
    let (_, result) = parser(input)?;

    Ok(result)
}

fn main() -> io::Result<()> {
    let f = File::open("GX010003.bin")?;
    println!("{:#?}", parse_metadata(f));

    Ok(())
}