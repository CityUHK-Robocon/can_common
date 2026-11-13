// Host self-check. Run from the repo root:
//   c++ -std=c++11 -Itests/stub -Isrc tests/host_test.cpp src/can_common.cpp -o /tmp/can_common_test && /tmp/can_common_test
#include <can_common.h>
#include <assert.h>
#include <stdio.h>

struct Fake : CAN_COMMON {
    uint32_t id, mask; bool ext;
    Fake(int n) : CAN_COMMON(n), id(0), mask(0), ext(false) {}
    int _setFilterSpecific(uint8_t, uint32_t, uint32_t, bool) { return 0; }
    int _setFilter(uint32_t i, uint32_t m, bool e) { id = i; mask = m; ext = e; return 7; }
    uint32_t init(uint32_t) { return 0; }
    uint32_t beginAutoSpeed() { return 0; }
    uint32_t set_baudrate(uint32_t) { return 0; }
    void setListenOnlyMode(bool) {}
    void enable() {}
    void disable() {}
    bool sendFrame(CAN_FRAME&) { return true; }
    bool rx_avail() { return false; }
    uint16_t available() { return 0; }
    uint32_t get_rx_buff(CAN_FRAME&) { return 0; }
    int filters() { return numFilters; }
    bool anyCallback() {
        for (int i = 0; i < CAN_MAX_FILTERS; i++) if (cbCANFrame[i] || cbCANFrameFD[i]) return true;
        return false;
    }
};

static void cb(CAN_FRAME *) {}

int main() {
    // const and non-const bit reads agree for every bit
    CAN_FRAME f;
    f.data.uint64 = 0xA5C3F00F12345678ull;
    const CAN_FRAME &cf = f;
    for (int i = 0; i < 64; i++) {
        bool want = (f.data.uint8[i / 8] >> (i % 8)) & 1;
        assert(cf.data.bit[i] == want);
        assert((bool)f.data.bit[i] == want);
    }
    CAN_FRAME_FD fd;
    fd.data.uint8[63] = 0x80;
    fd.data.uint8[5] = 0x04;
    const CAN_FRAME_FD &cfd = fd;
    assert(cfd.data.bit[511] && cfd.data.bit[42] && !cfd.data.bit[41] && !cfd.data.bit[510]);

    // constructor clamps the filter count and clears every callback slot
    Fake big(100), neg(-3);
    assert(big.filters() == CAN_MAX_FILTERS && neg.filters() == 0);
    assert(!big.anyCallback());
    big.setCallback(31, cb);
    big.setCallback(32, cb); // out of range, must be ignored
    assert(big.anyCallback());
    big.removeCallback();
    assert(!big.anyCallback());

    // explicit watchFor reaches the driver untouched
    Fake k(8);
    assert(k.watchFor(0x123, 0x1FFFFFFF, true) == 7);
    assert(k.id == 0x123 && k.mask == 0x1FFFFFFF && k.ext);

    // watchForRange matches the old exhaustive loop on small ranges
    for (uint32_t a = 0; a < 0x900; a += 7) {
        for (uint32_t b = a; b < a + 300; b += 13) {
            uint32_t id = a, mask = (b <= 0x7FF) ? 0x7FF : 0x1FFFFFFF;
            for (uint32_t c = a; c <= b; c++) { id &= c; mask &= (~(a ^ c)) & 0x1FFFFFFF; }
            k.watchForRange(b, a); // reversed on purpose
            assert(k.id == id && k.mask == mask);
            assert(k.ext == (b > 0x7FF));
            for (uint32_t c = a; c <= b; c++) assert((c & k.mask) == k.id);
        }
    }
    // huge ranges return immediately instead of looping
    k.watchForRange(0, 0x1FFFFFFF);
    assert(k.id == 0 && k.mask == 0 && k.ext);
    k.watchForRange(0x100, 0x1000);
    assert(k.ext && k.id == 0 && k.mask == 0x1FFFE000);

    puts("host_test: ok");
    return 0;
}
