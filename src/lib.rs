#![no_std]

extern crate embedded_hal as hal;

use hal::digital::OutputPin;
use hal::digital::InputPin;
use hal::blocking::delay::DelayUs;


pub trait OpenDrainOutput: OutputPin + InputPin {}
impl<P: OutputPin + InputPin> OpenDrainOutput for P {}

const DHT_BYTES: usize = 5;

#[derive(Debug)]
pub struct Response {
    data: [u8; DHT_BYTES],
}

pub enum Type {
    DHT22,
}

impl Response {
    pub fn decode_temperature(&self, _kind: Type) -> f32 {
        let value = (self.data[2] as i16) << 8 | self.data[3] as i16;
        let value = if value < 0 { -(value & 0x7fff) } else { value };
        (value as f32) / 10f32
    }

    pub fn decode_humidity(&self, _kind: Type) -> f32 {
        let value = (self.data[0] as u16) << 8 | self.data[1] as u16;
        (value as f32) / 10f32
    }
}

#[derive(Debug)]
pub enum Error {
    WaitForStateTimeout{ state: bool, timeout_us: u8 },
    ChecksumMismatch{ data: [u8; DHT_BYTES], min: u8, max: u8 },
}

pub struct DHT<'a> {
    output: &'a mut OpenDrainOutput,
}

impl<'a> DHT<'a> {
    pub fn new(output: &'a mut OpenDrainOutput) -> DHT<'a> {
        Self {
            output,
        }
    }

    pub fn read(&mut self, delay: &mut DelayUs<u16>) -> Result<Response, Error> {
        self.output.set_low();
        delay.delay_us(1000);
        self.output.set_high();

        self.wait_for_state(false, 40, delay)?; // 20-40us  before LOW  response
        self.wait_for_state(true, 80, delay)?;  // 80us     before HIGH response
        self.wait_for_state(false, 80, delay)?; // 80us     before data transfer starts

        const DHT_BITS: usize = DHT_BYTES * 8;
        let mut times: [u8; DHT_BITS] = unsafe { core::mem::uninitialized() };
        let mut min = 255u8;
        let mut max = 0u8;
        for time_ref in times.iter_mut() {
            self.wait_for_state(true, 50, delay)?; // 50us  before HIGH state
            let time = self.wait_for_state(false, 100, delay)?; // 25-70us to next bit
            *time_ref = time;
            max = core::cmp::max(max, time);
            min = core::cmp::min(min, time);
        }
        let threshold = ((min as u16 + max as u16) / 2) as u8;

        let mut data: [u8; DHT_BYTES] = unsafe { core::mem::uninitialized() };
        for (i, byte_ref) in data.iter_mut().enumerate() {
            let si = i * 8;
            let mut byte: u8 = 0;
            for time in &times[si .. si + 8] {
                byte = (byte << 1) | if *time >= threshold { 1 } else { 0 };
            }
            *byte_ref = byte;
        }
        let checksum = data[..4].iter()
            .fold(0, |sum, b| b + sum);
        if checksum != data[4] {
            Err(Error::ChecksumMismatch{ data, min, max })
        } else {
            Ok(Response{ data })
        }
    }

    fn wait_for_state(&self, state: bool, timeout_us: u8, delay: &mut DelayUs<u16>) -> Result<u8, Error> {
        for tick in 0..timeout_us {
            if self.output.is_high() == state {
                return Ok(tick);
            }
            delay.delay_us(1);
        }
        Err(Error::WaitForStateTimeout{ state, timeout_us })
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn response_subzero() {
        let d = ::Response{ data: [0x02, 0xf9, 0x80, 0x1c, 0x97] };
        assert_eq!(d.decode_temperature(::Type::DHT22), -2.8);
    }
}
