
#[no_mangle]
pub extern "C" fn _sbrk() {}

#[no_mangle]
pub extern "C" fn _write() {}

#[no_mangle]
pub extern "C" fn _close() {}

#[no_mangle]
pub extern "C" fn _lseek() {}

#[no_mangle]
pub extern "C" fn _read() -> i32 { -1 }

#[no_mangle]
pub extern "C" fn _fstat() -> i32 { -1 }

#[no_mangle]
pub extern "C" fn _isatty() {}

#[no_mangle]
pub extern "C" fn _exit() {}

#[no_mangle]
pub extern "C" fn _open() -> i32 { -1 }

#[no_mangle]
pub extern "C" fn _kill() {}

#[no_mangle]
pub extern "C" fn _getpid() {}
