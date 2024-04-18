use alloc::boxed::Box;
use core::fmt::Write;

use log::{Level, Log, Metadata, Record};

pub struct SerialLogger {
    pub level: Level,
    pub writer: Option<Box<dyn Write>>,
    pub filters: &'static [&'static str],
}

unsafe impl Sync for SerialLogger {}
unsafe impl Send for SerialLogger {}

static mut SERIAL_LOGGER: SerialLogger = SerialLogger {
    level: Level::Info,
    writer: None,
    filters: &[],
};

impl SerialLogger {
    pub fn init(level: Level, w: Box<dyn Write>, f: &'static [&'static str]) {
        unsafe {
            // Set global SERIAL_LOGGER log context
            SERIAL_LOGGER.level = level;
            SERIAL_LOGGER.writer = Some(w);
            SERIAL_LOGGER.filters = f;

            // Attach to logger
            let _ = log::set_logger(&SERIAL_LOGGER);
            log::set_max_level(SERIAL_LOGGER.level.to_level_filter());
        }
    }
}

impl Log for SerialLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= self.level
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let _ = unsafe {
                if SERIAL_LOGGER
                    .filters
                    .iter()
                    .find(|f| Some(**f) == record.module_path())
                    .is_some()
                {
                    return;
                }

                match record.module_path() {
                    Some(p) => write!(
                        SERIAL_LOGGER,
                        "[{}] {} - {}\r\n",
                        record.level(),
                        p,
                        record.args()
                    ),
                    None => write!(
                        SERIAL_LOGGER,
                        "[{}] - {}\r\n",
                        record.level(),
                        record.args()
                    ),
                }
            };
        }
    }

    fn flush(&self) {}
}

impl core::fmt::Write for SerialLogger {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // Fetch writer (if bound)
        let w = match &mut self.writer {
            Some(w) => w,
            None => return Err(core::fmt::Error),
        };

        w.write_str(s).map_err(|_| core::fmt::Error)?;

        Ok(())
    }
}
