use core::fmt::Debug;
use std::io;
use std::io::prelude::*;
use std::fs::File;


use nom::error::ErrorKind;
use nom::bytes::streaming::{tag, take};
use nom::number::streaming::{be_u8, be_u16, be_u32};
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

enum Block {
    DEVC([u8; 3]),
    DVID([u8; 4]),
    DVNM(String),
    STRM,
    STMP,
    TSMP,
    STNM(String),
    ORIN,
    SIUN
}

fn parse_devc(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(&[0])(input)?;
    let (input, device_source) = take(3usize)(input)?;

    let mut device_source_array = [0u8; 3];
    device_source_array.copy_from_slice(device_source);

    Ok((input, Block::DEVC(device_source_array)))
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

    Ok((input, Block::DVID(device_id_array)))
}

fn parse_dvnm(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"c")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let string_length = (size as usize)*(count as usize);
    let (input, device_name) = take(string_length)(input)?;
    let device_name = std::str::from_utf8(device_name).unwrap();
    println!("Got device name \"{}\"", device_name);

    // Take remaining padding bytes
    let (input, _padding) = if string_length % 4 != 0 {
        let count_remaining_bytes = 4 - string_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::DVNM(device_name.to_string())))
}

fn parse_strm(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(&[0])(input)?;
    let (input, _) = take(3usize)(input)?; // Stream ID?

    Ok((input, Block::STRM))
}

fn parse_stmp(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"J")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let length = (size as usize)*(count as usize);
    let (input, _unknown) = take(length)(input)?;

    // Take remaining padding bytes
    let (input, _padding) = if length % 4 != 0 {
        let count_remaining_bytes = 4 - length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::STMP))
}


fn parse_tsmp(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"L")(input)?;
    let (input, size) = be_u8(input)?;
    assert_eq!(size, 4);
    let (input, count) = be_u16(input)?;
    assert_eq!(count, 1);

    let (input, total_samples) = be_u32(input)?;
    println!("Total samples: {}", total_samples);

    Ok((input, Block::TSMP))
}


fn parse_stnm(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"c")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let string_length = (size as usize)*(count as usize);
    let (input, stream_name) = take(string_length)(input)?;
    let stream_name = std::str::from_utf8(stream_name).unwrap();
    println!("Got stream name \"{}\"", stream_name);

    // Take remaining padding bytes
    let (input, _padding) = if string_length % 4 != 0 {
        let count_remaining_bytes = 4 - string_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::STNM(stream_name.to_string())))
}

fn parse_orin(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"c")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let string_length = (size as usize)*(count as usize);
    let (input, orin) = take(string_length)(input)?;
    let orin = std::str::from_utf8(orin).unwrap();
    println!("Got ORIN \"{}\"", orin);

    // Take remaining padding bytes
    let (input, _padding) = if string_length % 4 != 0 {
        let count_remaining_bytes = 4 - string_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::ORIN))
}

fn parse_siun(input: &[u8]) -> IResult<&[u8], Block> {
    let (input, _data_type) = tag(b"c")(input)?;
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    let string_length = ((size - 1) as usize)*(count as usize); // SI units seem to have an out-of-range byte at the end. Maybe means ^-2?
    let (input, si_units) = take(string_length)(input)?;
    let si_units = std::str::from_utf8(si_units).unwrap();
    println!("Got SI units \"{}\"", si_units);

    // Take remaining padding bytes
    let (input, _padding) = if string_length % 4 != 0 {
        let count_remaining_bytes = 4 - string_length % 4;
        take(count_remaining_bytes)(input)?
    } else {
        (input, &[][..])
    };

    Ok((input, Block::SIUN))
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
        block_type => {
            println!("Got unexpected block type {:x?} | {:?}", block_type, std::str::from_utf8(block_type).unwrap());
            Err(nom::Err::Failure(nom::error::Error::new(input, ErrorKind::Tag)))
        }
    }?;

    Ok((input, block))
}

fn parser(input: &[u8]) -> IResult<&[u8], ()> {
    let mut input = input;
    loop {
        let result = parse_block(input)?;
        input = result.0;
    }
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

fn parse_metadata<T: Read>(mut f: T) -> Result<Vec<u8>, ParseError> {
    let mut buffer = [0u8; 1024];
    let bytes_read = f.read(&mut buffer)?;
    // let mut buffer = Vec::new();
    // let bytes_read = f.read_to_end(&mut buffer)?;

    let input = &buffer[..bytes_read];
    let (input, result) = parser(input)?;

    Ok(vec![])
}

fn main() -> io::Result<()> {
    let f = File::open("GX010003.bin")?;
    println!("{:#?}", parse_metadata(f));

    Ok(())
}