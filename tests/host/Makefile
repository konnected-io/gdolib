# Host-side tests: compile the real gdolib sources against the fake ESP-IDF in stubs/.
#   make -C tests/host          build and run
#   GDO_TEST_LOG=1 make -C tests/host    also print gdolib's ESP_LOGx output

ROOT := ../..
CC ?= cc
CFLAGS ?= -std=gnu11 -g -O0 -Wall -Wextra -Wno-unused-function -Wno-unused-parameter \
          -Wno-missing-field-initializers -Wno-enum-conversion -fsanitize=address,undefined
CPPFLAGS := -Istubs -I$(ROOT)/include -I$(ROOT)

BUILD := build
TESTS := $(patsubst %.c,$(BUILD)/%,$(wildcard test_*.c))
LIB_SRCS := idf_fake.c $(ROOT)/gdo_utils.c $(ROOT)/secplus.c

.PHONY: test clean
test: $(TESTS)
	@rc=0; for t in $(TESTS); do echo "== $$t"; ./$$t || rc=1; done; exit $$rc

$(BUILD)/%: %.c $(LIB_SRCS) $(ROOT)/gdo.c $(ROOT)/gdo_priv.h $(ROOT)/include/gdo.h stubs/idf_fake.h harness.h
	@mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LIB_SRCS)

clean:
	rm -rf $(BUILD)
