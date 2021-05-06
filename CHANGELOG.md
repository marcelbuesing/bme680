# Change Log
## [0.6.0](https://github.com/marcelbuesing/bme680/tree/0.6.0) (2021-05-06)
[Full Changelog](https://github.com/marcelbuesing/bme680/compare/0.5.1..0.6.0)
- @jgosmann Add missing sleep to example. Closes #23.
  One has to sleep for the duration of the configured profile to read
  proper data.
- @jgosmann Expose gas_valid and heat_stab.
  The BME680 datasheet recommends to check these values to ensure that a
  gas reading is valid. To do so, they must be exposed by the library.
- @huntc avoid retaining ownership of delayer, by passing the `delayer` as mutable reference.
- @huntc make Doctests work on non linux environments.
- Update influx example dependency to tokio 1.5

## [0.5.1](https://github.com/marcelbuesing/bme680/tree/0.5.1) (2020-04-05)
[Full Changelog](https://github.com/marcelbuesing/bme680/compare/0.5.0..0.5.1)
- Update futures-timer and linux-embedded hal dependencies
- Cleanup clippy warnings

## [0.5.0](https://github.com/marcelbuesing/bme680/tree/0.5.0) (2019-11-27)
[Full Changelog](https://github.com/marcelbuesing/bme680/compare/0.4.3..0.5.0)
- Migrate example to futures 0.3 and toko 0.2.
- Migrate crate to Rust 2018 edition.

## [0.4.3](https://github.com/marcelbuesing/bme680/tree/0.4.3) (2018-03-19)
[Full Changelog](https://github.com/marcelbuesing/bme680/compare/0.4.2..0.4.3)
- @caemor replaced non no_std compatible `f32.signum` and `f32.abs`. Fixes #14.

## [0.4.2](https://github.com/marcelbuesing/bme680/tree/0.4.2) (2018-01-05)
[Full Changelog](https://github.com/marcelbuesing/bme680/compare/0.4.1..0.4.2)
- `get_sensor_settings` now properly returns the configured temperature offset, closes #12.
  Thank you @nickbroon for reporting this and @caemor for the feedback.

## [0.4.1](https://github.com/marcelbuesing/bme680/tree/0.4.1) (2018-12-22)
[Full Changelog](https://github.com/marcelbuesing/bme680/compare/0.4.0..0.4.1)
- `with_temperature_offset` now uses `f32` instead of`i32`. Fixed calulation for non negative offsets.

## [0.4.0](https://github.com/marcelbuesing/bme680/tree/0.4.0) (2018-12-18)
[Full Changelog](https://github.com/marcelbuesing/bme680/compare/0.3.2..0.4.0)
- Add `with_temperature_offset` to `SettingsBuilder` for specifying a temperature offset for the temperature calculation.

## [0.3.2](https://github.com/marcelbuesing/bme680/tree/0.3.2) (2018-11-12)
[Full Changelog](https://github.com/marcelbuesing/bme680/compare/0.3.1..0.3.2)
- Add screenshots to documentation
- Removed Cargo.lock
- Update dependencies:
  - env-logger to `v0.6`
  - influent to `v0.5`

**Merged pull requests:**
- Use no_std in reading temperatures example [\#4](https://github.com/marcelbuesing/bme680/pull/4) ([chrysn](https://github.com/chrysn))