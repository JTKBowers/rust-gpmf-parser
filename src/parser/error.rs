use std::fmt::Debug;

#[derive(Debug)]
pub enum ParseError {
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