use core::fmt::Debug;
use std::io;
use std::io::prelude::*;
use std::fs::File;


use nom::error::ErrorKind;
use nom::bytes::streaming::{tag, take};
use nom::number::streaming::{be_u8, be_u16, be_u32, be_u64, be_i16, be_f32};
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
    DeviceSource([u8; 4]),
    DeviceID([u8; 4]),
    DeviceName(String),
    Stream([u8; 4]),
    StartTimestamp(u64),
    TotalSamples(u32),
    StreamName(String),
    InputOrientation(String),
    UnitsSI(String),
    ScalingFactor(i16),
    Temperature(f32),
    Acceleration(Vec<[i16; 3]>)
}

fn parse_devc(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, device_source) = take(4usize)(input)?;

    let mut device_source_array = [0u8; 4];
    device_source_array.copy_from_slice(device_source);

    Ok((input, Block::DeviceSource(device_source_array)))
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
    let (input, stream_id_slice) = take(4usize)(input)?;

    let mut stream_id = [0u8; 4];
    stream_id.copy_from_slice(stream_id_slice);

    Ok((input, Block::Stream(stream_id)))
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

    let string_length = ((size - 1) as usize)*(count as usize); // SI units seem to have an out-of-range byte at the end. Maybe means ^-2?
    let (input, si_units) = take(string_length)(input)?;
    let si_units = std::str::from_utf8(si_units).unwrap();

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
    let (input, _data_type) = tag(b"s")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 2);
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let (input, scaling_factor) = be_i16(input)?;

    let (input, _) = take(2usize)(input)?;

    Ok((input, Block::ScalingFactor(scaling_factor)))
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
        b"SCAL" => parse_scal(input),
        b"TMPC" => parse_tmpc(input),
        b"ACCL" => parse_accl(input),
        block_type => {
            println!("Got unexpected block type {:x?} | {:?}", block_type, std::str::from_utf8(block_type).unwrap());
            Err(nom::Err::Failure(nom::error::Error::new(input, ErrorKind::Tag)))
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
        println!("{:?}", result.1);
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
    let mut buffer = [0u8; 2048];
    let bytes_read = f.read(&mut buffer)?;
    // let mut buffer = Vec::new();
    // let bytes_read = f.read_to_end(&mut buffer)?;

    let input = &buffer[..bytes_read];
    let (_, result) = parser(input)?;

    Ok(result)
}

fn main() -> io::Result<()> {
    let f = File::open("GX010003.bin")?;
    println!("{:#?}", parse_metadata(f));

    Ok(())
}