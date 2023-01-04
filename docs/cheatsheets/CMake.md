---
title: CMake
category: Ubuntu
layout: 2017/sheet
tags: [Featured]
updated: 2023-01-04
weight: -10
intro: CMake
---



### パッケージの探索結果で分岐する

`find_package(pkg_name)`の結果は`pkg_name_FOUND`に出力されるのを利用する.
REQUIRED指定すると見つからなかった場合にビルドエラーになっていしまうので注意

```CMakeLists.txt
find_package(Boost)
if( Boost_FOUND )
  # do something
endif()
```

