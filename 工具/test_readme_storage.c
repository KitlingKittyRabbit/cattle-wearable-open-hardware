/* Exercise the same transactional README/FAT state machine used by main.c. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../固件/源码/issue3_logic.h"

#define MOCK_FIRST_CLUSTER 2U
#define MOCK_CLUSTER_COUNT 1400U
#define MOCK_FAT_SECTORS ((MOCK_CLUSTER_COUNT * 4U + 511U) / 512U)
#define MOCK_SPC 1U

typedef struct {
    uint8_t root[512];
    uint8_t fat[MOCK_FAT_SECTORS][512];
    uint8_t fat_cache[512];
    uint8_t *data;
    Issue3FatScan scan;
    uint32_t fat_read_calls;
    uint32_t budget_returns;
    uint32_t root_write_calls;
    uint32_t data_write_calls;
    uint32_t fail_fat_read_call;
    uint32_t fail_data_write_call;
    uint32_t fail_root_write_call;
} MockStorage;

static void put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void fat_set(MockStorage *m, uint32_t cluster, uint32_t value)
{
    uint32_t byte = cluster * 4U;
    assert(byte / 512U < MOCK_FAT_SECTORS);
    put32(&m->fat[byte / 512U][byte % 512U], value);
}

static uint32_t fat_get(const MockStorage *m, uint32_t cluster)
{
    uint32_t byte = cluster * 4U;
    return get32(&m->fat[byte / 512U][byte % 512U]) & 0x0FFFFFFFU;
}

static int read_root(void *ctx, uint8_t *sector)
{
    memcpy(sector, ((MockStorage *)ctx)->root, 512U);
    return 0;
}

static int write_root(void *ctx, const uint8_t *sector)
{
    MockStorage *m = (MockStorage *)ctx;
    m->root_write_calls++;
    memcpy(m->root, sector, 512U);
    if (m->fail_root_write_call != 0U &&
        m->root_write_calls == m->fail_root_write_call)
        return -1; /* simulate a partial directory-sector write */
    return 0;
}

static int fat_read(void *ctx, uint32_t lba, uint8_t *sector)
{
    MockStorage *m = (MockStorage *)ctx;
    m->fat_read_calls++;
    if (m->fail_fat_read_call != 0U &&
        m->fat_read_calls == m->fail_fat_read_call)
        return -1;
    if (lba >= MOCK_FAT_SECTORS) return -1;
    memcpy(sector, m->fat[lba], 512U);
    return 0;
}

static int allocate_cluster(void *ctx, uint32_t *cluster)
{
    MockStorage *m = (MockStorage *)ctx;
    int result = issue3_fat_scan_step(&m->scan, MOCK_FIRST_CLUSTER,
                                      MOCK_CLUSTER_COUNT, 0U, 1U,
                                      fat_read, m, m->fat_cache, cluster);
    if (result == ISSUE3_FAT_BUDGET) m->budget_returns++;
    if (result == ISSUE3_FAT_FOUND) {
        fat_set(m, *cluster, 0x0FFFFFFFU);
        put32(&m->fat_cache[(*cluster * 4U) % 512U], 0x0FFFFFFFU);
    }
    return result;
}

static int set_fat(void *ctx, uint32_t cluster, uint32_t value)
{
    fat_set((MockStorage *)ctx, cluster, value);
    return 0;
}

static int free_chain(void *ctx, uint32_t first_cluster)
{
    MockStorage *m = (MockStorage *)ctx;
    uint32_t cluster = first_cluster;
    for (uint32_t i = 0U; i < MOCK_CLUSTER_COUNT; i++) {
        if (cluster < MOCK_FIRST_CLUSTER ||
            cluster >= MOCK_FIRST_CLUSTER + MOCK_CLUSTER_COUNT)
            return 0;
        uint32_t next = fat_get(m, cluster);
        fat_set(m, cluster, 0U);
        if (next < MOCK_FIRST_CLUSTER || next >= 0x0FFFFFF8U) return 0;
        cluster = next;
    }
    return -1;
}

static int write_data(void *ctx, uint32_t cluster, uint32_t sector_in_cluster,
                      const uint8_t *sector)
{
    MockStorage *m = (MockStorage *)ctx;
    m->data_write_calls++;
    assert(cluster < MOCK_FIRST_CLUSTER + MOCK_CLUSTER_COUNT);
    memcpy(&m->data[cluster * 512U + sector_in_cluster * 512U], sector, 512U);
    if (m->fail_data_write_call != 0U &&
        m->data_write_calls == m->fail_data_write_call)
        return -1;
    return 0;
}

static void reset_mock(MockStorage *m)
{
    memset(m, 0, sizeof(*m));
    m->data = (uint8_t *)calloc(MOCK_FIRST_CLUSTER + MOCK_CLUSTER_COUNT,
                                512U);
    assert(m->data != NULL);
    m->scan.cursor = MOCK_FIRST_CLUSTER;
    fat_set(m, 0U, 0x0FFFFFFFU);
    fat_set(m, 1U, 0x0FFFFFFFU);
}

static void free_mock(MockStorage *m)
{
    free(m->data);
    m->data = NULL;
}

static Issue3ReadmeStorage ops_for(MockStorage *m)
{
    Issue3ReadmeStorage ops = {
        read_root, write_root, allocate_cluster, set_fat, free_chain,
        write_data, m, MOCK_FIRST_CLUSTER, MOCK_CLUSTER_COUNT, MOCK_SPC
    };
    return ops;
}

static uint16_t readme_entry(const MockStorage *m)
{
    uint8_t name[11];
    issue3_readme_name(name);
    for (uint16_t off = 0U; off < 512U; off += 32U) {
        if (memcmp(&m->root[off], name, 11U) == 0) return off;
    }
    return 0xFFFFU;
}

static uint32_t entry_cluster(const MockStorage *m, uint16_t off)
{
    return ((uint32_t)m->root[off + 20U] << 16) |
           ((uint32_t)m->root[off + 21U] << 24) |
           (uint32_t)m->root[off + 26U] |
           ((uint32_t)m->root[off + 27U] << 8);
}

static void seed_old_readme(MockStorage *m)
{
    uint8_t name[11];
    issue3_readme_name(name);
    memcpy(&m->root[0], "LOG0001 BIN", 11U);
    memcpy(&m->root[32], "LOG0002 BIN", 11U);
    m->root[11] = m->root[43] = 0x20;
    memcpy(&m->root[64], name, 11U);
    m->root[75] = 0x20;
    m->root[64U + 26U] = 20U;
    m->root[64U + 28U] = 17U;
    fat_set(m, 20U, 21U);
    fat_set(m, 21U, 0x0FFFFFFFU);
}

static void occupy_before_fat_sector(MockStorage *m, uint32_t sector)
{
    uint32_t first_free = sector * 128U;
    for (uint32_t cluster = MOCK_FIRST_CLUSTER; cluster < first_free;
         cluster++)
        fat_set(m, cluster, 0x0FFFFFFFU);
}

static void assert_content(const MockStorage *m, uint32_t first_cluster)
{
    uint8_t expected[512];
    assert(issue3_readme_copy_sector(expected, 0U) == 512U);
    assert(memcmp(&m->data[first_cluster * 512U], expected, 512U) == 0);
    assert(issue3_readme_copy_sector(expected, 1U) == 241U);
    assert(memcmp(&m->data[(first_cluster + 1U) * 512U], expected, 512U) == 0);
}

static void test_success_after_occupied_prefix(void)
{
    for (uint32_t sector = 0U; sector < 9U; sector++) {
        MockStorage m;
        uint8_t root_buf[512], data_buf[512];
        reset_mock(&m);
        occupy_before_fat_sector(&m, sector);
        Issue3ReadmeStorage ops = ops_for(&m);
        assert(issue3_readme_write_transaction(&ops, root_buf, data_buf) ==
               ISSUE3_README_OK);
        assert(m.budget_returns == sector);
        uint16_t entry = readme_entry(&m);
        assert(entry != 0xFFFFU);
        uint32_t first = entry_cluster(&m, entry);
        assert(first >= sector * 128U);
        assert(fat_get(&m, first) == first + 1U);
        assert(fat_get(&m, first + 1U) >= 0x0FFFFFF8U);
        assert_content(&m, first);
        assert(get32(&m.root[entry + 28U]) == issue3_readme_length());
        free_mock(&m);
    }
}

static void test_full_and_io_are_distinct(void)
{
    MockStorage full, io;
    uint8_t root_buf[512], data_buf[512], before[512];
    reset_mock(&full);
    for (uint32_t cluster = MOCK_FIRST_CLUSTER;
         cluster < MOCK_FIRST_CLUSTER + MOCK_CLUSTER_COUNT; cluster++)
        fat_set(&full, cluster, 0x0FFFFFFFU);
    memcpy(before, full.root, sizeof(before));
    Issue3ReadmeStorage full_ops = ops_for(&full);
    assert(issue3_readme_write_transaction(&full_ops, root_buf, data_buf) ==
           ISSUE3_README_FULL);
    assert(memcmp(before, full.root, sizeof(before)) == 0);
    free_mock(&full);

    reset_mock(&io);
    io.fail_fat_read_call = 1U;
    Issue3ReadmeStorage io_ops = ops_for(&io);
    assert(issue3_readme_write_transaction(&io_ops, root_buf, data_buf) ==
           ISSUE3_README_IO);
    assert(readme_entry(&io) == 0xFFFFU);
    free_mock(&io);
}

static void test_data_and_directory_rollback_preserve_old_chain(void)
{
    for (uint8_t failure_kind = 0U; failure_kind < 2U; failure_kind++) {
        MockStorage m;
        uint8_t root_buf[512], data_buf[512], before[512];
        reset_mock(&m);
        seed_old_readme(&m);
        memcpy(before, m.root, sizeof(before));
        if (failure_kind == 0U) m.fail_data_write_call = 2U;
        else m.fail_root_write_call = 1U;
        Issue3ReadmeStorage ops = ops_for(&m);
        assert(issue3_readme_write_transaction(&ops, root_buf, data_buf) ==
               ISSUE3_README_IO);
        assert(memcmp(before, m.root, sizeof(before)) == 0);
        assert(fat_get(&m, 20U) == 21U && fat_get(&m, 21U) >= 0x0FFFFFF8U);
        for (uint32_t cluster = 2U; cluster < 4U; cluster++)
            assert(fat_get(&m, cluster) == 0U);
        free_mock(&m);
    }
}

static void test_replace_reclaims_old_chain(void)
{
    MockStorage m;
    uint8_t root_buf[512], data_buf[512];
    reset_mock(&m);
    seed_old_readme(&m);
    uint8_t previous_entries[64];
    memcpy(previous_entries, m.root, sizeof(previous_entries));
    Issue3ReadmeStorage ops = ops_for(&m);
    assert(issue3_readme_write_transaction(&ops, root_buf, data_buf) ==
           ISSUE3_README_OK);
    uint16_t entry = readme_entry(&m);
    uint32_t first = entry_cluster(&m, entry);
    assert(first == 2U && fat_get(&m, first) == first + 1U);
    assert(fat_get(&m, 20U) == 0U && fat_get(&m, 21U) == 0U);
    assert(memcmp(previous_entries, m.root, sizeof(previous_entries)) == 0);
    assert_content(&m, first);
    free_mock(&m);
}

int main(void)
{
    uint8_t name[11];
    issue3_readme_name(name);
    assert(memcmp(name, "README  TXT", 11U) == 0);
    assert(issue3_readme_length() == 753U);
    assert(issue3_readme_sector_count() == 2U);
    test_success_after_occupied_prefix();
    test_full_and_io_are_distinct();
    test_data_and_directory_rollback_preserve_old_chain();
    test_replace_reclaims_old_chain();
    puts("README 8.3、跨扇区、FAT BUDGET/FULL/IO、失败回滚：通过");
    return 0;
}
