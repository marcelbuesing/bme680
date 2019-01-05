# Change Log
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