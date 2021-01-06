use nom::number::streaming::{be_u16, be_u8};
use nom::IResult;

pub fn parse_size_count(input: &[u8]) -> IResult<&[u8], (usize, usize)> {
    let (input, size) = be_u8(input)?;
    let (input, count) = be_u16(input)?;

    Ok((input, (size as usize, count as usize)))
}
