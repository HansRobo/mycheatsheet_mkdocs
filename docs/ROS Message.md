---
title: ROS Message
category: ROS
layout: 2017/sheet
tags:
  - ROS
updated: 2023-01-28
weight: -10
intro: ROS Message
---

## ??\_msgs/msg/details フォルダの中身は何なのか？

- xx\_\_builder.hpp
  - メッセージの構築処理
- xx\_\_functions.h/c
  - コピー・等号関数の定義など
- xx\_\_rosidl_type_support_\<dds\>\_c/ccp.h/hpp
  - シリアライズ・デシリアライズ関数
- xx\_\_struct.h/hpp
  - structの定義、equal/ not equal演算子の定義
- xx\_\_traits.hpp
  - yamlへの変換関数の定義
- xx\_\_type_support.h/hpp/c
