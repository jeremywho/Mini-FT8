#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "test_ft8_synth_fixtures.h"
#include "ft8_tx_synth.h"

static std::vector<uint8_t> run_python_reference(const uint8_t* tones, float base_hz) {
    // Write tones as hex
    char hex[2 * FT8_TX_SYNTH_SYMBOLS + 1] = {0};
    for (int i = 0; i < FT8_TX_SYNTH_SYMBOLS; ++i) {
        snprintf(hex + i * 2, 3, "%02x", tones[i]);
    }
    // Build command — on Windows, use `python` (the launcher); on POSIX, `python3` is standard.
    // We're targeting Windows-via-Git-Bash for this dev workflow.
    char cmd[1024];
    snprintf(cmd, sizeof(cmd),
        "python ../tools/ft8_synth_reference.py %s %g _synth_ref_tmp.bin",
        hex, base_hz);
    int rc = system(cmd);
    if (rc != 0) {
        fprintf(stderr, "python reference failed (rc=%d): %s\n", rc, cmd);
        std::exit(2);
    }
    FILE* f = fopen("_synth_ref_tmp.bin", "rb");
    if (!f) { perror("ref"); std::exit(2); }
    std::vector<uint8_t> v(FT8_TX_SYNTH_SAMPLES);
    size_t n = fread(v.data(), 1, FT8_TX_SYNTH_SAMPLES, f);
    fclose(f);
    remove("_synth_ref_tmp.bin");
    if (n != FT8_TX_SYNTH_SAMPLES) {
        fprintf(stderr, "ref read short: %zu (expected %d)\n", n, FT8_TX_SYNTH_SAMPLES);
        std::exit(2);
    }
    return v;
}

static int compare_byte_exact(const char* name, const std::vector<uint8_t>& a,
                              const std::vector<uint8_t>& b) {
    if (a.size() != b.size()) {
        printf("[%s] FAIL: size %zu vs %zu\n", name, a.size(), b.size());
        return 1;
    }
    int diffs = 0;
    int first_diff = -1;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            if (first_diff < 0) first_diff = (int)i;
            ++diffs;
        }
    }
    if (diffs == 0) {
        printf("[%s] PASS: %zu bytes match\n", name, a.size());
        return 0;
    }
    printf("[%s] FAIL: %d byte diffs, first at index %d (got 0x%02X, expected 0x%02X)\n",
           name, diffs, first_diff, a[first_diff], b[first_diff]);
    return 1;
}

static int test_fixture(const char* name, const uint8_t* tones) {
    std::vector<uint8_t> got(FT8_TX_SYNTH_SAMPLES);
    ft8_tx_synth_render(tones, 1500.0f, got.data(), false);
    std::vector<uint8_t> ref = run_python_reference(tones, 1500.0f);
    return compare_byte_exact(name, got, ref);
}

static int test_byte_stuffing(const char* name, const uint8_t* tones) {
    std::vector<uint8_t> stuffed(FT8_TX_SYNTH_SAMPLES);
    ft8_tx_synth_render(tones, 1500.0f, stuffed.data(), true);
    int count_3b = 0;
    for (uint8_t b : stuffed) if (b == 0x3B) ++count_3b;
    if (count_3b == 0) {
        printf("[%s/stuff] PASS: no 0x3B bytes in stuffed output\n", name);
        return 0;
    }
    printf("[%s/stuff] FAIL: %d 0x3B bytes present\n", name, count_3b);
    return 1;
}

int main(int argc, char* argv[]) {
    (void)argc; (void)argv;
    int fails = 0;
    fails += test_fixture("cq", fixture_cq);
    fails += test_fixture("report", fixture_report);
    fails += test_fixture("73", fixture_73);
    fails += test_byte_stuffing("cq", fixture_cq);
    fails += test_byte_stuffing("report", fixture_report);
    fails += test_byte_stuffing("73", fixture_73);
    if (fails == 0) printf("ALL OK\n");
    return fails ? 1 : 0;
}
