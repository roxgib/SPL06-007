[![Crates.io Version][crates-io-badge]][crates-io]
[![Crates.io Downloads][crates-io-download-badge]][crates-io-download]
![No Std][no-std-badge]

An I2C driver for the SPL06-007 barometric pressure and temperature sensor, intended for use in embedded environments.

## Usage

The driver is designed to be used with the embedded-hal and requires an I2C interface to be passed to the driver. The driver is generic over the I2C interface and the error type, allowing it to be used with any I2C implementation so long as it supports the required traits.

Add the following to your Cargo.toml:

```toml
[dependencies]
spl06_007 = "0.3"
```

Example usage on an Arduino Uno:

```rust
#![no_std]
#![no_main]

use arduino_hal::prelude::*;
use panic_halt as _;

use spl06_007::Barometer;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().expect("Failed to take peripherals");
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let mut i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    );

    let mut barometer = Barometer::new(&mut i2c).expect("Failed to instantiate barometer");
    barometer.init().expect("Failed to initialise barometer");

    loop {
        ufmt::uwriteln!(&mut serial, "T: {:?}", barometer.get_temperature().unwrap() as u16).void_unwrap();
        ufmt::uwriteln!(&mut serial, "P: {:?}", barometer.get_pressure().unwrap() as u16).void_unwrap();
        ufmt::uwriteln!(&mut serial, "A: {:?}", barometer.altitude(1020.0).unwrap() as u16).void_unwrap();
    }
}
```

It is necessary to call `init` before any other methods are called. This method will set some default values for the sensor and is suitable for most use cases. Alternatively you can set the mode, sample rate, and oversampling values manually:

```rust
barometer.set_pressure_config(SampleRate::Single, SampleRate::Eight);
barometer.set_temperature_config(SampleRate::Single, SampleRate::Eight);
barometer.set_mode(Mode::ContinuousPressureTemperature);
```

This is useful if you want to change the sample rate or oversampling values, such as for more rapid updates, better precsion, or lower power draw. It is also possible to set the mode to `Mode::Standby` to reduce power consumption, and request a single measurement at a time with `Barometer::get_temperature_blocking` and `Barometer::get_pressure_blocking`.

<!-- Badges -->
[crates-io]: https://crates.io/crates/spl06-007
[crates-io-badge]: https://img.shields.io/crates/v/spl06-007.svg?maxAge=3600
[crates-io-download]: https://crates.io/crates/spl06-007
[crates-io-download-badge]: https://img.shields.io/crates/d/spl06-007.svg?maxAge=3600
[no-std-badge]: https://img.shields.io/badge/no__std-yes-blue