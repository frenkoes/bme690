# bme690
A `no_std` async BME690 driver written in Rust.

`Bme690` is the standard async driver and accepts `embedded-hal-async` I2C
and delay implementations. `AsyncBme690` remains an alias for `Bme690`.

```rust
use bme690::Bme690;

let mut sensor = Bme690::new(i2c, 0x76, delay);
sensor.init().await?;
let data = sensor.read_sensor_data().await?;
```


