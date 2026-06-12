# forks/refloat/tests/Makefile — host-gcc harness for the pure AGR cores
CC ?= gcc
CFLAGS := -Wall -Wextra -Werror -std=c11 -D_GNU_SOURCE -g -I./host_compat -I../src
CORE_SRC :=
CORE_SRC += ../src/agr_geometry.c
CORE_SRC += ../src/agr_profile.c
CORE_SRC += ../src/agr_control.c
CORE_SRC += ../src/filters/ema.c
TESTS := $(patsubst %.c,%,$(wildcard test_*.c))

all: $(TESTS)

%: %.c $(CORE_SRC)
	$(CC) $(CFLAGS) -o $@ $< $(CORE_SRC) -lm

run: all
	@fail=0; for t in $(TESTS); do echo "== $$t"; ./$$t || fail=1; done; exit $$fail

clean:
	rm -f $(TESTS)

.PHONY: all run clean
