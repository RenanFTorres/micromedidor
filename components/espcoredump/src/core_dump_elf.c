/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "esp_attr.h"
#include "esp_partition.h"
#include "esp_flash_encrypt.h"
#include "sdkconfig.h"
#include "core_dump_checksum.h"
#include "core_dump_elf.h"
#include "esp_core_dump_port.h"
#include "esp_core_dump_port_impl.h"
#include "esp_core_dump_common.h"

#ifdef CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
#include "esp_app_desc.h"
#endif

#define ELF_CLASS ELFCLASS32

#include "elf.h"                    // for ELF file types

#define ELF_SEG_HEADERS_COUNT(_self_) ((_self_)->segs_count)

#define ELF_HLEN 52
#define ELF_CORE_SEC_TYPE 1
#define ELF_PR_STATUS_SEG_NUM 0
#define ELF_ESP_CORE_DUMP_INFO_TYPE 8266
#define ELF_ESP_CORE_DUMP_EXTRA_INFO_TYPE 677
#define ELF_NOTE_NAME_MAX_SIZE 32
#define ELF_APP_SHA256_SIZE 66

#define ELF_CHECK_ERR(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_COREDUMP_LOGE("%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

typedef enum
{
    ELF_STAGE_CALC_SPACE = 0,
    ELF_STAGE_PLACE_HEADERS = 1,
    ELF_STAGE_PLACE_DATA = 2
} core_dump_elf_stages_t;

typedef enum _elf_err_t
{
    ELF_PROC_ERR_SKIP_HEADER = 0,
    ELF_PROC_ERR_STACK_CORRUPTED = -1,
    ELF_PROC_ERR_WRITE_FAIL = -2,
    ELF_PROC_ERR_OTHER = -3
} core_dump_elf_proc_err_t;

typedef struct _core_dump_task_info_t
{
    elf_phdr* phdr;
    void* frame;
    core_dump_task_header_t* task_hdr;
    uint32_t task_id;
    size_t tcb_sz;
    int* size_ptr;
} core_dump_task_data_t;

typedef struct
{
    uint32_t version; // coredump version
    uint8_t app_elf_sha256[ELF_APP_SHA256_SIZE]; // sha256 of elf file
} core_dump_elf_version_info_t;

const static char TAG[] __attribute__((unused)) = "esp_core_dump_elf";

// Main ELF handle type
typedef struct _core_dump_elf_t
{
    core_dump_elf_version_info_t    elf_version_info;
    uint16_t                        elf_stage;
    uint32_t                        elf_next_data_offset;
    uint16_t                        segs_count;
    core_dump_write_config_t *      write_cfg;
} core_dump_elf_t;

// Represents lightweight implementation to save core dump data into ELF formatted binary

#define ALIGN(b, var) var = align(b, var)

#if CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF

static inline uint32_t align(uint32_t width, uint32_t in)
{
  return (in + (width - 1)) & -width;
}

// Builds elf header and check all data offsets
static int elf_write_file_header(core_dump_elf_t *self, uint32_t seg_count)
{
    elfhdr elf_hdr; // declare as static to save stack space

    if (self->elf_stage == ELF_STAGE_PLACE_HEADERS) {
        ESP_COREDUMP_LOG_PROCESS("Segment count %u", seg_count);
        memset(&elf_hdr, 0, sizeof(elfhdr));
        elf_hdr.e_ident[0] = ELFMAG0;
        elf_hdr.e_ident[1] = ELFMAG1;
        elf_hdr.e_ident[2] = ELFMAG2;
        elf_hdr.e_ident[3] = ELFMAG3;
        elf_hdr.e_ident[4] = ELFCLASS32;
        elf_hdr.e_ident[5] = ELFDATA2LSB;
        elf_hdr.e_ident[6] = EV_CURRENT;
        elf_hdr.e_ident[7] = ELFOSABI_NONE;
        elf_hdr.e_ident[8] = 0;
        elf_hdr.e_type = ET_CORE;
        elf_hdr.e_machine = esp_core_dump_get_arch_id();
        elf_hdr.e_flags = 0;
        elf_hdr.e_version = EV_CURRENT;
        elf_hdr.e_entry = 0;
        _Static_assert(sizeof(elfhdr) == ELF_HLEN, "Invalid ELF header struct length!");
        elf_hdr.e_phoff = sizeof(elfhdr);          // program header table's file offset in bytes.
        elf_hdr.e_phentsize = sizeof(elf_phdr);    // size in bytes of one entry in the file program header table
        elf_hdr.e_phnum = seg_count;                 // number of program segments
        elf_hdr.e_shoff = 0;                       // section header table's file offset in bytes.
        elf_hdr.e_ehsize = sizeof(elfhdr);         // elf header size
        elf_hdr.e_shentsize = sizeof(elf_shdr);    // section header's size in bytes.
        elf_hdr.e_shnum = 0;                       // initial section counter is 0
        elf_hdr.e_shstrndx = SHN_UNDEF;            // do not use string table
        // write built elf header into elf image
        esp_err_t err = self->write_cfg->write(self->write_cfg->priv, (void*)&elf_hdr, sizeof(elf_hdr));
        ELF_CHECK_ERR((err == ESP_OK), ELF_PROC_ERR_WRITE_FAIL,
                        "Write ELF header failure (%d)", err);
        ESP_COREDUMP_LOG_PROCESS("Add file header %u bytes", sizeof(elf_hdr));
    }

    return self->elf_stage == ELF_STAGE_PLACE_DATA ? 0 : sizeof(elf_hdr);
}

static int elf_write_segment_header(core_dump_elf_t *self, elf_phdr* phdr)
{
    ELF_CHECK_ERR(phdr, ELF_PROC_ERR_SKIP_HEADER,
                    "Header is skipped, stage=(%d).", self->elf_stage);

    phdr->p_offset = self->elf_next_data_offset;
    // set segment data information and write it into image
    esp_err_t err = self->write_cfg->write(self->write_cfg->priv, (void*)phdr, sizeof(elf_phdr));
    ELF_CHECK_ERR((err == ESP_OK), ELF_PROC_ERR_WRITE_FAIL,
                    "Write ELF segment header failure (%d)", err);
    ESP_COREDUMP_LOG_PROCESS("Add segment header %u bytes: type %d, sz %u, off = 0x%x",
                            sizeof(elf_phdr), phdr->p_type, phdr->p_filesz, phdr->p_offset);

    return sizeof(elf_phdr);
}

static int elf_add_segment(core_dump_elf_t *self,
                            uint32_t type, uint32_t vaddr,
                            void* data, uint32_t data_sz)
{
    esp_err_t err = ESP_FAIL;
    elf_phdr seg_hdr = { 0 };
    int data_len = data_sz;

    ELF_CHECK_ERR((data != NULL), ELF_PROC_ERR_OTHER,
                "Invalid data for segment.");

    ALIGN(4, data_len);

    if (self->elf_stage == ELF_STAGE_CALC_SPACE) {
        self->segs_count++;
        return data_len + sizeof(elf_phdr);
    }
    if (self->elf_stage == ELF_STAGE_PLACE_HEADERS) {
        seg_hdr.p_type = type;
        seg_hdr.p_vaddr = vaddr;
        seg_hdr.p_paddr = vaddr;
        seg_hdr.p_filesz = data_len;
        seg_hdr.p_memsz = data_len;
        seg_hdr.p_flags = (PF_R | PF_W);
        int ret = elf_write_segment_header(self, &seg_hdr);
        ELF_CHECK_ERR((ret > 0), ret,
                        "Write ELF segment data failure (%d)", ret);
        self->elf_next_data_offset += data_len;
        return ret;
    }
    ESP_COREDUMP_LOG_PROCESS("Add segment size=%u, start_off=0x%x",
                                (uint32_t)data_len, self->elf_next_data_offset);
    // write segment data only when write function is set and phdr = NULL
    // write data into segment
    err = self->write_cfg->write(self->write_cfg->priv, data, (uint32_t)data_len);
    ELF_CHECK_ERR((err == ESP_OK), ELF_PROC_ERR_WRITE_FAIL,
                    "Write ELF segment data failure (%d)", err);
    self->elf_next_data_offset += data_len;
    return data_len;
}

static int elf_write_note(core_dump_elf_t *self,
                            const char* name,
                            uint32_t type,
                            void* data,
                            uint32_t data_sz)
{
    esp_err_t err = ESP_FAIL;
    // temporary buffer for note name
    static char name_buffer[ELF_NOTE_NAME_MAX_SIZE] = { 0 };
    elf_note note_hdr = { 0 };
    uint32_t name_len = strlen(name) + 1; // get name length including terminator
    uint32_t data_len = data_sz;

    ELF_CHECK_ERR(data, ELF_PROC_ERR_OTHER,
            "Invalid data pointer %x.", (uint32_t)data);
    ELF_CHECK_ERR((name_len <= ELF_NOTE_NAME_MAX_SIZE), 0,
                "Segment note name is too long %d.", name_len);

    ALIGN(4, data_len);
    ALIGN(4, name_len);
    uint32_t note_size = name_len + data_len + sizeof(elf_note);
    ALIGN(4, note_size);

    // write segment data during second pass
    if (self->elf_stage == ELF_STAGE_PLACE_DATA) {
        memcpy((void*)name_buffer, (void*)name, name_len);
        note_hdr.n_namesz = name_len;
        note_hdr.n_descsz = data_sz;
        note_hdr.n_type = type;
        // write note header
        err = self->write_cfg->write(self->write_cfg->priv, (void*)&note_hdr, sizeof(note_hdr));
        ELF_CHECK_ERR((err == ESP_OK), ELF_PROC_ERR_WRITE_FAIL,
                "Write ELF note header failure (%d)", err);
        // write note name
        err = self->write_cfg->write(self->write_cfg->priv, (void*)name_buffer, name_len);
        ELF_CHECK_ERR((err == ESP_OK), ELF_PROC_ERR_WRITE_FAIL,
                        "Write ELF note name failure (%d)", err);
        // write note data
        err = self->write_cfg->write(self->write_cfg->priv, (void*)data, data_len);
        ELF_CHECK_ERR((err == ESP_OK), ELF_PROC_ERR_WRITE_FAIL,
                        "Write ELF note data failure (%d)", err);
        ESP_COREDUMP_LOG_PROCESS("Add note size=%d, start_off=0x%x",
                                    note_size, self->elf_next_data_offset);
    }
    return note_size; // return actual note size
}

static int elf_add_note(core_dump_elf_t *self,
                        const char* name,
                        uint32_t type,
                        void* data,
                        uint32_t data_sz)
{
    ELF_CHECK_ERR((data != NULL), ELF_PROC_ERR_OTHER,
            "Invalid data pointer for segment");

    int note_size = elf_write_note(self, name, type, data, data_sz);
    ELF_CHECK_ERR((note_size > 0), note_size,
                    "Write ELF note data failure, returned (%d)", note_size);
    return note_size; // return actual note segment size
}

// Append note with registers dump to segment note
static int elf_add_regs(core_dump_elf_t *self, core_dump_task_header_t *task)
{
    void *reg_dump;

    uint32_t len = esp_core_dump_get_task_regs_dump(task, &reg_dump);
    if (len == 0) {
        ESP_COREDUMP_LOGE("Zero size register dump for task 0x%x!", task->tcb_addr);
        return ELF_PROC_ERR_OTHER;
    }

    // append note data with dump to existing note
    return elf_add_note(self,
                        "CORE",                // note name
                        ELF_CORE_SEC_TYPE,     // note type for reg dump
                        reg_dump,      // register dump with pr_status
                        len);
}

static int elf_add_stack(core_dump_elf_t *self, core_dump_task_header_t *task)
{
    uint32_t stack_vaddr, stack_len = 0, stack_paddr = 0;

    ELF_CHECK_ERR((task), ELF_PROC_ERR_OTHER, "Invalid task pointer.");

    stack_len = esp_core_dump_get_stack(task, &stack_vaddr, &stack_paddr);
    ESP_COREDUMP_LOG_PROCESS("Add stack for task 0x%x: addr 0x%x, sz %u",
                                task->tcb_addr, stack_vaddr, stack_len);
    int ret = elf_add_segment(self, PT_LOAD,
                                (uint32_t)stack_vaddr,
                                (void*)stack_paddr,
                                (uint32_t) stack_len);
    return ret;
}

static int elf_add_tcb(core_dump_elf_t *self, core_dump_task_header_t *task)
{
    ELF_CHECK_ERR((task), ELF_PROC_ERR_OTHER, "Invalid task pointer.");
    // add task tcb data into program segment of ELF
    ESP_COREDUMP_LOG_PROCESS("Add TCB for task 0x%x: addr 0x%x, sz %u",
                                task->tcb_addr, task->tcb_addr,
                                esp_core_dump_get_tcb_len());
    int ret = elf_add_segment(self, PT_LOAD,
                                (uint32_t)task->tcb_addr,
                                task->tcb_addr,
                                esp_core_dump_get_tcb_len());
    return ret;
}

static int elf_process_task_tcb(core_dump_elf_t *self, core_dump_task_header_t *task)
{
    int ret = ELF_PROC_ERR_OTHER;

    ELF_CHECK_ERR((task), ELF_PROC_ERR_OTHER, "Invalid input data.");

    // save tcb of the task as is and apply segment size
    ret = elf_add_tcb(self, task);
    if (ret <= 0) {
        ESP_COREDUMP_LOGE("Task (TCB:%x) processing failure = %d",
                                            task->tcb_addr,
                                            ret);
    }
    return ret;
}

static int elf_process_task_stack(core_dump_elf_t *self, core_dump_task_header_t *task)
{
    int ret = ELF_PROC_ERR_OTHER;

    ELF_CHECK_ERR((task), ELF_PROC_ERR_OTHER, "Invalid input data.");

    ret = elf_add_stack(self, task);
    if (ret <= 0) {
        ESP_COREDUMP_LOGE("Task (TCB:%x), (Stack:%x), stack processing failure = %d.",
                                    task->tcb_addr,
                                    task->stack_start,
                                    ret);
    }
    return ret;
}

static int elf_process_note_segment(core_dump_elf_t *self, int notes_size)
{
    int ret;
    elf_phdr seg_hdr = { 0 };

    if (self->elf_stage == ELF_STAGE_PLACE_HEADERS) {
        // segment header for PR_STATUS notes
        seg_hdr.p_type = PT_NOTE;
        seg_hdr.p_vaddr = 0;
        seg_hdr.p_paddr = 0;
        seg_hdr.p_filesz = notes_size;
        seg_hdr.p_memsz = notes_size;
        seg_hdr.p_flags = (PF_R | PF_W);
        ret = elf_write_segment_header(self, &seg_hdr);
        ELF_CHECK_ERR((ret > 0), ret, "NOTE segment header write failure, returned (%d).", ret);
        self->elf_next_data_offset += notes_size;
        return sizeof(seg_hdr);
    } else if (self->elf_stage == ELF_STAGE_CALC_SPACE) {
        self->segs_count++;
        notes_size += sizeof(seg_hdr);
    } else {
        // in "Place Data" phase segment body is been already filled by other functions
        ESP_COREDUMP_LOG_PROCESS("Add NOTE segment, size=%d, start_off=0x%x",
                                    notes_size, self->elf_next_data_offset);
        self->elf_next_data_offset += notes_size;
    }
    return (int)notes_size;
}

static int elf_process_tasks_regs(core_dump_elf_t *self)
{
    core_dump_task_header_t task_hdr = { 0 };
    void *task = NULL;
    int len = 0;
    int ret = 0;

    esp_core_dump_reset_tasks_snapshots_iter();
    task = esp_core_dump_get_current_task_handle();
    if (esp_core_dump_get_task_snapshot(task, &task_hdr, NULL)) {
        // place current task dump first
        ret = elf_add_regs(self,  &task_hdr);
        if (self->elf_stage == ELF_STAGE_PLACE_HEADERS) {
            // when writing segments headers this function writes nothing
            ELF_CHECK_ERR((ret >= 0), ret, "Task %x, PR_STATUS write failed, return (%d).", task, ret);
        } else {
            ELF_CHECK_ERR((ret > 0), ret, "Task %x, PR_STATUS write failed, return (%d).", task, ret);
        }
        len += ret;
    }
    // processes PR_STATUS and register dump for each task
    // each call to the processing function appends PR_STATUS note into note segment
    // and writes data or updates the segment note header accordingly (if phdr is set)
    task = NULL;
    while ((task = esp_core_dump_get_next_task(task))) {
        if (task == esp_core_dump_get_current_task_handle()) {
            continue; // skip current task (already processed)
        }
        if (esp_core_dump_get_task_snapshot(task, &task_hdr, NULL)) {
            ret = elf_add_regs(self,  &task_hdr);
            if (self->elf_stage == ELF_STAGE_PLACE_HEADERS) {
                // when writing segments headers this function writes nothing
                ELF_CHECK_ERR((ret >= 0), ret, "Task %x, PR_STATUS write failed, return (%d).", task, ret);
            } else {
                ELF_CHECK_ERR((ret > 0), ret, "Task %x, PR_STATUS write failed, return (%d).", task, ret);
            }
            len += ret;
        }
    }
    ret = elf_process_note_segment(self, len); // tasks regs note
    ELF_CHECK_ERR((ret > 0), ret,
                    "PR_STATUS note segment processing failure, returned(%d).", ret);
    return ret;
}

static int elf_save_task(core_dump_elf_t *self, core_dump_task_header_t *task)
{
    int elf_len = 0;

    int ret = elf_process_task_tcb(self, task);
    ELF_CHECK_ERR((ret > 0), ret,
                    "Task %x, TCB write failed, return (%d).", task->tcb_addr, ret);
    elf_len += ret;
    ret = elf_process_task_stack(self, task);
    ELF_CHECK_ERR((ret != ELF_PROC_ERR_WRITE_FAIL), ELF_PROC_ERR_WRITE_FAIL,
                    "Task %x, stack write failed, return (%d).", task->tcb_addr, ret);
    elf_len += ret;
    return elf_len;
}

static int elf_write_tasks_data(core_dump_elf_t *self)
{
    int elf_len = 0;
    void *task = NULL;
    core_dump_task_header_t task_hdr = { 0 };
    core_dump_mem_seg_header_t interrupted_stack = { 0 };
    int ret = ELF_PROC_ERR_OTHER;
    uint16_t tasks_num = 0;
    uint16_t bad_tasks_num = 0;

    ESP_COREDUMP_LOG_PROCESS("================ Processing task registers ================");
    ret = elf_process_tasks_regs(self);
    ELF_CHECK_ERR((ret > 0), ret, "Tasks regs addition failed, return (%d).", ret);
    elf_len += ret;

    ESP_COREDUMP_LOG_PROCESS("================   Processing task data   ================");
    // processes all task's stack data and writes segment data into partition
    // if flash configuration is set
    task = NULL;
    esp_core_dump_reset_tasks_snapshots_iter();
    while ((task = esp_core_dump_get_next_task(task))) {
        tasks_num++;
        if (!esp_core_dump_get_task_snapshot(task, &task_hdr, &interrupted_stack)) {
            bad_tasks_num++;
            continue;
        }
        ret = elf_save_task(self, &task_hdr);
        ELF_CHECK_ERR((ret > 0), ret,
                        "Task %x, TCB write failed, return (%d).", task, ret);
        elf_len += ret;
        if (interrupted_stack.size > 0) {
            ESP_COREDUMP_LOG_PROCESS("Add interrupted task stack %lu bytes @ %x",
                    interrupted_stack.size, interrupted_stack.start);
            ret = elf_add_segment(self, PT_LOAD,
                                    (uint32_t)interrupted_stack.start,
                                    (void*)interrupted_stack.start,
                                    (uint32_t)interrupted_stack.size);
            ELF_CHECK_ERR((ret > 0), ret, "Interrupted task stack write failed, return (%d).", ret);
            elf_len += ret;
        }
    }
    ESP_COREDUMP_LOG_PROCESS("Found %d bad task out of %d", bad_tasks_num, tasks_num);
    return elf_len;
}

static int elf_write_core_dump_user_data(core_dump_elf_t *self)
{
    int data_len = 0;
    int total_sz = 0;
    uint32_t start = 0;

    for (coredump_region_t i = COREDUMP_MEMORY_START; i < COREDUMP_MEMORY_MAX; i++) {
        data_len = esp_core_dump_get_user_ram_info(i, &start);

        ELF_CHECK_ERR((data_len >= 0), ELF_PROC_ERR_OTHER, "invalid memory region");

        if (data_len > 0) {
            int ret = elf_add_segment(self, PT_LOAD,
                        (uint32_t)start,
                        (void*)start,
                        (uint32_t) data_len);

            ELF_CHECK_ERR((ret > 0), ret, "memory region write failed. Returned (%d).", ret);
            total_sz += ret;
        }
    }

    return total_sz;
}

static int elf_write_core_dump_info(core_dump_elf_t *self)
{
    void *extra_info = NULL;

    ESP_COREDUMP_LOG_PROCESS("================ Processing coredump info ================");
    int data_len = (int)sizeof(self->elf_version_info.app_elf_sha256);
    data_len = esp_app_get_elf_sha256((char*)self->elf_version_info.app_elf_sha256, (size_t)data_len);
    ESP_COREDUMP_LOG_PROCESS("Application SHA256='%s', length=%d.",
                                self->elf_version_info.app_elf_sha256, data_len);
    self->elf_version_info.version = esp_core_dump_elf_version();
    int ret = elf_add_note(self,
                            "ESP_CORE_DUMP_INFO",
                            ELF_ESP_CORE_DUMP_INFO_TYPE,
                            &self->elf_version_info,
                            sizeof(self->elf_version_info));
    ELF_CHECK_ERR((ret > 0), ret, "Version info note write failed. Returned (%d).", ret);
    data_len = ret;

    uint32_t extra_info_len = esp_core_dump_get_extra_info(&extra_info);
    if (extra_info_len == 0) {
        ESP_COREDUMP_LOGE("Zero size extra info!");
        return ELF_PROC_ERR_OTHER;
    }

    ret = elf_add_note(self,
                        "EXTRA_INFO",
                        ELF_ESP_CORE_DUMP_EXTRA_INFO_TYPE,
                        extra_info,
                        extra_info_len);
    ELF_CHECK_ERR((ret > 0), ret, "Extra info note write failed. Returned (%d).", ret);
    data_len += ret;

    ret = elf_process_note_segment(self, data_len);
    ELF_CHECK_ERR((ret > 0), ret,
                    "EXTRA_INFO note segment processing failure, returned(%d).", ret);
    return ret;
}

static int esp_core_dump_do_write_elf_pass(core_dump_elf_t *self)
{
    int tot_len = 0;

    int data_sz = elf_write_file_header(self, ELF_SEG_HEADERS_COUNT(self));
    if (self->elf_stage == ELF_STAGE_PLACE_DATA) {
        ELF_CHECK_ERR((data_sz >= 0), data_sz, "ELF header writing error, returned (%d).", data_sz);
    } else {
        ELF_CHECK_ERR((data_sz > 0), data_sz, "ELF header writing error, returned (%d).", data_sz);
    }
    tot_len += data_sz;
    // Calculate whole size include headers for all tasks and main elf header
    data_sz = elf_write_tasks_data(self);
    ELF_CHECK_ERR((data_sz > 0), data_sz, "ELF Size writing error, returned (%d).", data_sz);
    tot_len += data_sz;

    // write core dump memory regions defined by user
    data_sz = elf_write_core_dump_user_data(self);
    ELF_CHECK_ERR((data_sz >= 0), data_sz, "memory regions writing error, returned (%d).", data_sz);
    tot_len += data_sz;

    // write data with version control information and some extra info
    // this should go after tasks processing
    data_sz = elf_write_core_dump_info(self);
    ELF_CHECK_ERR((data_sz > 0), data_sz, "Version info writing failed. Returned (%d).", data_sz);
    tot_len += data_sz;

    return tot_len;
}

esp_err_t esp_core_dump_write_elf(core_dump_write_config_t *write_cfg)
{
    static core_dump_elf_t self = { 0 };
    static core_dump_header_t dump_hdr = { 0 };
    esp_err_t err = ESP_OK;
    int tot_len = sizeof(dump_hdr);
    int write_len = sizeof(dump_hdr);

    ELF_CHECK_ERR((write_cfg), ESP_ERR_INVALID_ARG, "Invalid input data.");

    self.write_cfg = write_cfg;

    // On first pass (do not write actual data), but calculate data length needed to allocate memory
    self.elf_stage = ELF_STAGE_CALC_SPACE;
    ESP_COREDUMP_LOG_PROCESS("================= Calc data size ===============");
    int ret = esp_core_dump_do_write_elf_pass(&self);
    if (ret < 0) return ret;
    tot_len += ret;
    ESP_COREDUMP_LOG_PROCESS("Core dump tot_len=%lu", tot_len);
    ESP_COREDUMP_LOG_PROCESS("============== Data size = %d bytes ============", tot_len);

    // Prepare write elf
    if (write_cfg->prepare) {
        err = write_cfg->prepare(write_cfg->priv, (uint32_t*)&tot_len);
        if (err != ESP_OK) {
            ESP_COREDUMP_LOGE("Failed to prepare core dump storage (%d)!", err);
            return err;
        }
    }

    // Write start
    if (write_cfg->start) {
        err = write_cfg->start(write_cfg->priv);
        if (err != ESP_OK) {
            ESP_COREDUMP_LOGE("Failed to start core dump (%d)!", err);
            return err;
        }
    }

    // Write core dump header
    dump_hdr.data_len = tot_len;
    dump_hdr.version = esp_core_dump_elf_version();
    dump_hdr.tasks_num = 0; // unused in ELF format
    dump_hdr.tcb_sz = 0; // unused in ELF format
    dump_hdr.mem_segs_num = 0; // unused in ELF format
    err = write_cfg->write(write_cfg->priv,
                           (void*)&dump_hdr,
                           sizeof(core_dump_header_t));
    if (err != ESP_OK) {
        ESP_COREDUMP_LOGE("Failed to write core dump header (%d)!", err);
        return err;
    }

    self.elf_stage = ELF_STAGE_PLACE_HEADERS;
    // set initial offset to elf segments data area
    self.elf_next_data_offset = sizeof(elfhdr) + ELF_SEG_HEADERS_COUNT(&self) * sizeof(elf_phdr);
    ret = esp_core_dump_do_write_elf_pass(&self);
    if (ret < 0) return ret;
    write_len += ret;
    ESP_COREDUMP_LOG_PROCESS("============== Headers size = %d bytes ============", write_len);

    self.elf_stage = ELF_STAGE_PLACE_DATA;
    // set initial offset to elf segments data area, this is not necessary in this stage, just for pretty debug output
    self.elf_next_data_offset = sizeof(elfhdr) + ELF_SEG_HEADERS_COUNT(&self) * sizeof(elf_phdr);
    ret = esp_core_dump_do_write_elf_pass(&self);
    if (ret < 0) return ret;
    write_len += ret;
    ESP_COREDUMP_LOG_PROCESS("=========== Data written size = %d bytes ==========", write_len);

    // Write end, update checksum
    if (write_cfg->end) {
        err = write_cfg->end(write_cfg->priv);
        if (err != ESP_OK) {
            ESP_COREDUMP_LOGE("Failed to end core dump (%d)!", err);
            return err;
        }
    }
    return err;
}

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH

/* Below are the helper function to parse the core dump ELF stored in flash */

static esp_err_t elf_core_dump_image_mmap(esp_partition_mmap_handle_t* core_data_handle, const void **map_addr)
{
    size_t out_size;
    assert (core_data_handle);
    assert(map_addr);

    /* Find the partition that could potentially contain a (previous) core dump. */
    const esp_partition_t *core_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                                ESP_PARTITION_SUBTYPE_DATA_COREDUMP,
                                                                NULL);
    if (!core_part) {
        ESP_COREDUMP_LOGE("Core dump partition not found!");
        return ESP_ERR_NOT_FOUND;
    }
    if (core_part->size < sizeof(uint32_t)) {
        ESP_COREDUMP_LOGE("Core dump partition too small!");
        return ESP_ERR_INVALID_SIZE;
    }
    /* Data read from the mmapped core dump partition will be garbage if flash
     * encryption is enabled in hardware and core dump partition is not encrypted
     */
    if (esp_flash_encryption_enabled() && !core_part->encrypted) {
        ESP_COREDUMP_LOGE("Flash encryption enabled in hardware and core dump partition is not encrypted!");
        return ESP_ERR_NOT_SUPPORTED;
    }
    /* Read the size of the core dump file from the partition */
    esp_err_t ret = esp_partition_read(core_part, 0, &out_size, sizeof(uint32_t));
    if (ret != ESP_OK) {
        ESP_COREDUMP_LOGE("Failed to read core dump data size");
        return ret;
    }
    /* map the full core dump parition, including the checksum. */
    return esp_partition_mmap(core_part, 0, out_size, ESP_PARTITION_MMAP_DATA,
                              map_addr, core_data_handle);
}

static void elf_parse_version_info(esp_core_dump_summary_t *summary, void *data)
{
    core_dump_elf_version_info_t *version = (core_dump_elf_version_info_t *)data;
    summary->core_dump_version = version->version;
    memcpy(summary->app_elf_sha256, version->app_elf_sha256, ELF_APP_SHA256_SIZE);
    ESP_COREDUMP_LOGD("Core dump version 0x%x", summary->core_dump_version);
    ESP_COREDUMP_LOGD("App ELF SHA2 %s", (char *)summary->app_elf_sha256);
}

static void elf_parse_exc_task_name(esp_core_dump_summary_t *summary, void *tcb_data)
{
    StaticTask_t *tcb = (StaticTask_t *) tcb_data;
    /* An ugly way to get the task name. We could possibly use pcTaskGetName here.
     * But that has assumption that TCB pointer can be used as TaskHandle. So let's
     * keep it this way. */
    memset(summary->exc_task, 0, sizeof(summary->exc_task));
    strlcpy(summary->exc_task, (char *)tcb->ucDummy7, sizeof(summary->exc_task));
    ESP_COREDUMP_LOGD("Crashing task %s", summary->exc_task);
}

esp_err_t esp_core_dump_get_summary(esp_core_dump_summary_t *summary)
{
    int i;
    elf_phdr *ph;
    elf_note *note;
    const void *map_addr;
    size_t consumed_note_sz;
    esp_partition_mmap_handle_t core_data_handle;

    if (!summary) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = elf_core_dump_image_mmap(&core_data_handle, &map_addr);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t *ptr = (uint8_t *) map_addr + sizeof(core_dump_header_t);
    elfhdr *eh = (elfhdr *)ptr;

    ESP_COREDUMP_LOGD("ELF ident %02x %c %c %c", eh->e_ident[0], eh->e_ident[1], eh->e_ident[2], eh->e_ident[3]);
    ESP_COREDUMP_LOGD("Ph_num %d offset %x", eh->e_phnum, eh->e_phoff);

    for (i = 0; i < eh->e_phnum; i++) {
        ph = (elf_phdr *)((ptr + i * sizeof(*ph)) + eh->e_phoff);
        ESP_COREDUMP_LOGD("PHDR type %d off %x vaddr %x paddr %x filesz %x memsz %x flags %x align %x",
                          ph->p_type, ph->p_offset, ph->p_vaddr, ph->p_paddr, ph->p_filesz, ph->p_memsz,
                          ph->p_flags, ph->p_align);
        if (ph->p_type == PT_NOTE) {
            consumed_note_sz = 0;
            while(consumed_note_sz < ph->p_memsz) {
                note = (elf_note *)(ptr + ph->p_offset + consumed_note_sz);
                char *nm = (char *)(ptr + ph->p_offset + consumed_note_sz + sizeof(elf_note));
                ESP_COREDUMP_LOGD("Note NameSZ %x DescSZ %x Type %x name %s", note->n_namesz,
                                  note->n_descsz, note->n_type, nm);
                if (strncmp(nm, "EXTRA_INFO", note->n_namesz) == 0 ) {
                    esp_core_dump_summary_parse_extra_info(summary, (void *)(nm + note->n_namesz));
                }
                if (strncmp(nm, "ESP_CORE_DUMP_INFO", note->n_namesz) == 0 ) {
                    elf_parse_version_info(summary, (void *)(nm + note->n_namesz));
                }
                consumed_note_sz += note->n_namesz + note->n_descsz + sizeof(elf_note);
                ALIGN(4, consumed_note_sz);
            }
        }
    }
    /* Following code assumes that task stack segment follows the TCB segment for the respective task.
     * In general ELF does not impose any restrictions on segments' order so this can be changed without impacting core dump version.
     * More universal and flexible way would be to retrieve stack start address from crashed task TCB segment and then look for the stack segment with that address.
     */
    int flag = 0;
    for (i = 0; i < eh->e_phnum; i++) {
        ph = (elf_phdr *)((ptr + i * sizeof(*ph)) + eh->e_phoff);
        if (ph->p_type == PT_LOAD) {
            if (flag) {
                esp_core_dump_summary_parse_exc_regs(summary, (void *)(ptr + ph->p_offset));
                esp_core_dump_summary_parse_backtrace_info(&summary->exc_bt_info, (void *) ph->p_vaddr,
                                                           (void *)(ptr + ph->p_offset), ph->p_memsz);
                break;
            }
            if (ph->p_vaddr == summary->exc_tcb) {
                elf_parse_exc_task_name(summary, (void *)(ptr + ph->p_offset));
                flag = 1;
            }
        }
    }
    esp_partition_munmap(core_data_handle);
    return ESP_OK;
}

#endif // CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH

#endif //CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
