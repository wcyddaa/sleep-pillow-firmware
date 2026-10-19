// edge_aggregator.h - 转发到 include/edge_aggregator.h
// ============================================================
// 历史原因：从 v2.0 复制组件时,components/edge_aggregator/ 根目录
// 有一份同名头(原始版本,未做 IDF 5.5 适配)。
// GCC 在 #include "edge_aggregator.h" 时会优先在源文件所在目录查找,
// 命中根目录这份,导致 include/ 下的修改版根本没被读到。
//
// 转发到 include/edge_aggregator.h(已适配 IDF 5.5:void* task_ 等)
// 路径相对于当前文件: components/edge_aggregator/edge_aggregator.h
//                    -> components/edge_aggregator/include/edge_aggregator.h
// ============================================================
#pragma once
#include "include/edge_aggregator.h"
