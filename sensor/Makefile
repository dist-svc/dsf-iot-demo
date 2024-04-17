
ARGS=SODIUM_DISABLE_PIE=1 LDFLAGS='--specs=nosys.specs' CLAGS='-DLIBSODIUM_NO_PLATFORM_RANDOM=1' SODIUM_HOST=arm-none-eabi

build:
	${ARGS} cargo build --release

check:
	${ARGS} cargo check

watch-check:
	${ARGS} cargo watch -x check

run-pub:
	${ARGS} cargo run --bin dsf-pub --release -- --probe 066BFF504955657867073629

run-sub:
	${ARGS} cargo run --bin dsf-sub --release -- --probe 0671FF323338424E43111241

flash-pub:
	${ARGS} cargo flash --bin dsf-pub --release --probe 0483:374b:066BFF504955657867073629

flash-sub:
	${ARGS} cargo flash --bin dsf-sub --release --probe 0483:374b:0671FF323338424E43111241
