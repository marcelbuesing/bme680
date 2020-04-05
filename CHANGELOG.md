# Change Log
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