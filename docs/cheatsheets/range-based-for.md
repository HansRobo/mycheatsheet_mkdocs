---
title: 範囲for文
category: C++
layout: 2017/sheet
tags: [Featured]
updated: 2022-06-16
weight: -10
intro: 範囲for文の使い方
---

{: .-three-column}

### 範囲for文(range-based for)でインデックスを使いたい

`boost::adaptors::indexed`を使う
```c++
#include <boost/range/adaptor/indexed.hpp>
for (const auto & e : elements | boost::adaptors::indexed()) {
    std::cout << e.index() << ":" << e.value() << std::endl;
}
```

[range-v3](https://github.com/ericniebler/range-v3)ライブラリを使う
```c++
#include <range/v3/view/enumerate.hpp>
for (const auto & [index, value] : elements | ranges::views::enumerate) {
    std::cout << index << ":" << value << std::endl;
}
```

C++20以降で書ける方法
```cpp
for (size_t index = 0; const auto & e: elemnets) {
    std::cout << index << ":" << value << std::endl;
    ++index;
}
```
### 範囲for文の型の使い方
基本的にはユニバーサル参照 `auto &&` を使う
```cpp
for(auto && e : elements){}
```

配列に操作を加えない時はconst参照 `const auto  &` を使う
```cpp
for(const auto & e : elements){}
```

