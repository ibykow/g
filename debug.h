#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-macros"
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wgnu-zero-variadic-macro-arguments"

#ifndef DEBUG_H_
#define DEBUG_H_

/*
 * Some basic debug macros.
 * 2015, Ilia Bykow
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>

#define afprintf(f, ...) assert(fprintf(f, ##__VA_ARGS__) >= 0)

#define TO_STR(s) #s
#define STR(s) TO_STR(s)
#define LINE_STR__ STR(__LINE__)
#define FILE_LINE__ __FILE__ ":" LINE_STR__

#define TTY_RED     "\x1b[31m"
#define TTY_GREEN   "\x1b[32m"
#define TTY_YELLOW  "\x1b[33m"
#define TTY_BLUE    "\x1b[34m"
#define TTY_MAGENTA "\x1b[35m"
#define TTY_CYAN    "\x1b[36m"
#define TTY_RESET   "\x1b[0m"

#define COLOR(col, text) col text TTY_RESET

#define LOG(file, desc, fmt, ...) if (isatty(fileno(file))) {           \
    afprintf(file, desc " (%s in " COLOR(TTY_GREEN, FILE_LINE__) "): "\
        fmt "\n", __func__, ##__VA_ARGS__);                             \
} else {                                                                \
    afprintf(file, desc " (%s in " FILE_LINE__ "): " fmt "\n",        \
        __func__, ##__VA_ARGS__);                                       \
}

#define LOG_ERR(fmt, ...) \
    LOG(stderr, COLOR(TTY_RED, "ERROR"), fmt, ##__VA_ARGS__)

#define LOG_INFO(fmt, ...) \
    LOG(stdout, COLOR(TTY_GREEN, "INFO"), fmt, ##__VA_ARGS__)

#define LOG_PASS_ENABLED
#ifdef LOG_PASS_ENABLED
#define LOG_PASS(fmt, ...) \
    LOG(stdout, COLOR(TTY_GREEN, "PASS"), fmt, ##__VA_ARGS__)
#else
#define LOG_PASS(fmt, ...) NOP;
#endif /* end of ifdef: LOG_PASS_ENABLED */

#define LOG_FAIL(fmt, ...) \
    LOG(stdout, COLOR(TTY_RED, "FAIL"), fmt, ##__VA_ARGS__)

#define INFO LOG_INFO
#define ERR LOG_ERR

#define SHOW(expr, fmt, ...) INFO(COLOR(TTY_CYAN, STR(expr)) ": " fmt, (expr), ##__VA_ARGS__)

#define PRNL() puts("")

#define NOP (void) 0

#define TEST(expr, verbose_pass) do {     \
    test_total++;                   \
    if ((expr)) {                   \
        if (verbose_pass) {         \
            LOG_PASS("%s", STR(expr));    \
        }                           \
    } else{                         \
        test_failed++;              \
        LOG_FAIL("%s", STR(expr));  \
    }                               \
} while (0)

#define TEST_Q(expr) TEST(expr, 0)
#define TEST_V(expr) TEST(expr, 1)

#define TEST_SUMMARY() do { \
    if (test_total) { \
        LOG_INFO("Completed %zd test%s. %zd passed.", \
        test_total, test_total == 1 ? "" : "s", test_total - test_failed); \
    } else { \
        LOG_INFO("Skipped tests."); \
    } \
    test_total = 0; test_failed = 0; \
} while (0)

static ssize_t test_total = 0, test_failed = 0;

#endif /* end of include guard: _DEBUG_H_ */

#pragma clang diagnostic pop
#pragma clang diagnostic pop
