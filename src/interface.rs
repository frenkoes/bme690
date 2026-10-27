// src/interface.rs
// I2C register interface glue between `embedded-hal-async` and the generated
// `device-driver` register definitions in `registers.rs`.

use device_driver::{AsyncRegisterInterface, FieldsetMetadata, RegisterInterfaceBase};
use embedded_hal_async::i2c::{ErrorType, I2c as AsyncI2c};

/// Maximum register size (in bytes) used anywhere in `bme690.ddsl`.
/// Only used to size the stack buffer for combined address+data writes.
const MAX_REGISTER_LEN: usize = 3;

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
