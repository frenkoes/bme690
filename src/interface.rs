// src/interface.rs
// I2C register interface glue between `embedded-hal` and the generated
// `device-driver` register definitions in `registers.rs`.

use device_driver::{AsyncRegisterInterface, FieldsetMetadata, RegisterInterfaceBase};
use embedded_hal::i2c::{ErrorType, I2c};
use embedded_hal_async::i2c::I2c as AsyncI2c;

/// Maximum register size (in bytes) used anywhere in `bme690.ddsl`.
/// Only used to size the stack buffer for combined address+data writes.
const MAX_REGISTER_LEN: usize = 3;

pub(crate) struct BlockingAsync<T>(pub(crate) T);

impl<T: ErrorType> ErrorType for BlockingAsync<T> {
    type Error = T::Error;
}

impl<T: I2c> AsyncI2c for BlockingAsync<T> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.0.transaction(address, operations)
    }
}

impl<T: embedded_hal::delay::DelayNs> embedded_hal_async::delay::DelayNs for BlockingAsync<T> {
    async fn delay_ns(&mut self, ns: u32) {
        self.0.delay_ns(ns);
    }

    async fn delay_us(&mut self, us: u32) {
        self.0.delay_us(us);
    }

    async fn delay_ms(&mut self, ms: u32) {
        self.0.delay_ms(ms);
    }
}

pub struct I2cInterface<I2C> {
    pub(crate) i2c: I2C,
    pub(crate) address: u8,
}

impl<I2C> I2cInterface<I2C> {
    pub(crate) fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }
}

impl<I2C: ErrorType> RegisterInterfaceBase for I2cInterface<I2C> {
    type Error = I2C::Error;
    type AddressType = u8;
}

impl<I2C: AsyncI2c> AsyncRegisterInterface for I2cInterface<I2C> {
    async fn write_register(
        &mut self,
        address: u8,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        let mut buffer = [0u8; MAX_REGISTER_LEN + 1];
        buffer[0] = address;
        buffer[1..=data.len()].copy_from_slice(data);
        self.i2c.write(self.address, &buffer[..=data.len()]).await
    }

    async fn read_register(
        &mut self,
        address: u8,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[address], data).await
    }
}
