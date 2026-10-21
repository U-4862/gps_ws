#pragma once
/**
 * @file test.h
 * @brief Test 目录的统一入口头文件
 *
 * 原来这里是 `#include ""`，空路径会直接编译失败，已修掉。
 * 实际用的时候直接 include 具体头即可（TestChassis.hpp 已经带了 ChassisMsg.hpp）。
 */

#include "Chassis/ChassisMsg.hpp"
#include "TestChassis.hpp"
