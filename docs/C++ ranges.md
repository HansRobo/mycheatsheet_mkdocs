---
title: C++ ranges
category: C++
layout: 2017/sheet
tags: [C++, range-v3, Featured]
created: 2025-03-05 23:35:59
updated: 2025-11-29
weight: -10
intro: range-v3ライブラリの実践的cookbook
---

### セットアップ

**インクルード方法（個別）**

```cpp
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/algorithm/max.hpp>
#include <range/v3/algorithm/find_if.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/range/operations.hpp>
```

**名前空間エイリアス（推奨）**

```cpp
namespace rv = ranges::views;
namespace ra = ranges::actions;
```

---

## データフィルタリング

### 条件で要素を絞りたい

**基本的なfilter**

```cpp
auto available_items = items | ranges::views::filter([](const auto & item) {
    return item.is_available();
});
```

**複数条件の連鎖（AND条件）**

```cpp
auto filtered = items
    | ranges::views::filter([&](const auto & item) {
        return item.distance() >= 1.0;  // 条件1
    })
    | ranges::views::filter([&](const auto & item) {
        return item.score() > threshold;  // 条件2
    });
```

**filter + transformの連鎖**

```cpp
auto results = robots
    | ranges::views::filter([](const auto & r) { return r->available(); })
    | ranges::views::transform([&](const auto & r) {
        return calculate_score(r);
    });
```

---

## データ変換

### 要素を加工したい

**基本的なtransform**

```cpp
auto scores = items | ranges::views::transform([](const auto & item) {
    return item.value * 2.0;
});
```

**ペアに変換（要素 + 計算結果）**

```cpp
auto items_with_scores = items
    | ranges::views::transform([&](const auto & item) {
        double score = calculate_score(item);
        return std::make_pair(item, score);
    })
    | ranges::to<std::vector>();
```

**ネストしたtransform（2次元行列生成）**

```cpp
// ロボット位置からターゲットへの距離行列を生成
auto cost_matrix = robot_positions
    | ranges::views::transform([&](const auto & robot_pos) {
        return targets
            | ranges::views::transform([&](const auto & target) {
                return distance(robot_pos, target);
            })
            | ranges::to<std::vector>();
    })
    | ranges::to<std::vector>();
// 結果: vector<vector<double>>
```

---

## 最適値の取得

### 最小値/最大値を見つけたい

**値として取得（min / max）**

```cpp
auto min_value = ranges::min(values);
auto max_value = ranges::max(values);
```

**イテレータで取得（min_element / max_element）**

```cpp
auto best = ranges::max_element(items_with_scores,
    [](const auto & a, const auto & b) {
        return a.second < b.second;  // スコアで比較
    });

if (best != items_with_scores.end()) {
    auto [item, score] = *best;
    // 最大スコアの要素を使用
}
```

**空チェックとの組み合わせ（重要）**

ビューは遅延評価のため、空の場合にmin/maxを呼ぶとエラーになる。必ず`ranges::empty()`でチェック：

```cpp
auto score_view = items
    | ranges::views::filter([](const auto & item) { return item->available(); })
    | ranges::views::transform([&](const auto & item) {
        return calculate_score(item);
    });

// 空チェック必須！
auto result = ranges::empty(score_view) ? default_value : ranges::min(score_view);
```

---

## 要素の検索

### 条件に合う要素を探したい

**find_if**

```cpp
auto it = ranges::find_if(items, [&](const auto & item) {
    return item.id == target_id;
});

if (it != ranges::end(items)) {
    // 見つかった
    auto found = *it;
}
```

**contains（存在確認）**

```cpp
#include <range/v3/algorithm/contains.hpp>

if (ranges::contains(ids, target_id)) {
    // 含まれている
}
```

**count（個数カウント）**

```cpp
#include <range/v3/algorithm/count.hpp>

auto num_available = ranges::count_if(items, [](const auto & item) {
    return item.available();
});
```

---

## インデックス付きループ

### 位置（インデックス）も知りたい

**enumerate**

```cpp
#include <range/v3/view/enumerate.hpp>

for (const auto & [index, item] : ranges::views::enumerate(items)) {
    std::cout << index << ": " << item.name << std::endl;
}
```

**std::distanceからの移行**

```cpp
// Before: std::distance使用
for (auto it = items.begin(); it != items.end(); ++it) {
    size_t index = std::distance(items.begin(), it);
    // ...
}

// After: enumerate使用（推奨）
for (const auto & [index, item] : ranges::views::enumerate(items)) {
    // indexは0始まり
}
```

---

## シーケンス生成

### 数列を作りたい

**iota（数値範囲生成）**

```cpp
#include <range/v3/view/iota.hpp>

// 0, 1, 2, ..., 9
for (int i : ranges::views::iota(0, 10)) {
    // ...
}
```

**iotaとtransformの組み合わせ（等間隔ポイント生成）**

```cpp
int num_points = 100;
Point start_point = {0, 0};
Point end_point = {10, 0};

auto points_with_scores = ranges::views::iota(0, num_points)
    | ranges::views::transform([&](int i) -> Point {
        // 等間隔にポイントを生成
        double t = static_cast<double>(i) / num_points;
        return start_point + (end_point - start_point) * t;
    })
    | ranges::views::transform([&](const Point & p) {
        // 各ポイントのスコアを計算
        double score = evaluate(p);
        return std::make_pair(p, score);
    })
    | ranges::to<std::vector>();
```

---

## 部分取得

### 一部だけ欲しい

**drop（先頭N個スキップ）**

```cpp
#include <range/v3/view/drop.hpp>

// 最初の3個をスキップ
auto remaining = items | ranges::views::drop(3);
```

**take_while（条件を満たす間だけ）**

```cpp
#include <range/v3/view/take_while.hpp>

auto valid_items = items | ranges::views::take_while([](const auto & item) {
    return item.is_valid();
});
```

**take（先頭N個だけ）**

```cpp
#include <range/v3/view/take.hpp>

auto first_five = items | ranges::views::take(5);
```

---

## ビューの実体化

### vectorに変換したい

**to\<std::vector\>()**

```cpp
#include <range/v3/range/conversion.hpp>

// ビューをvectorに変換
std::vector<Item> result = items
    | ranges::views::filter([](const auto & item) { return item.valid(); })
    | ranges::to<std::vector>();
```

**いつ実体化すべきか**

| 状況 | 推奨 |
|------|------|
| 1回だけ走査する | ビューのまま（遅延評価） |
| 複数回走査する | to\<vector\>()で実体化 |
| ソートが必要 | 実体化してからsort |
| サイズが必要 | 実体化してから.size() |

```cpp
// 悪い例：毎回ビューを走査
auto view = items | ranges::views::filter(...);
for (int i = 0; i < 10; ++i) {
    for (const auto & item : view) { /* 毎回フィルタが実行される */ }
}

// 良い例：一度実体化して再利用
auto cached = items | ranges::views::filter(...) | ranges::to<std::vector>();
for (int i = 0; i < 10; ++i) {
    for (const auto & item : cached) { /* キャッシュから取得 */ }
}
```

---

## インプレース操作

### 元データを変更したい

**actions::sort**

```cpp
#include <range/v3/action/sort.hpp>

std::vector<int> values = {3, 1, 4, 1, 5};
values |= ranges::actions::sort;  // インプレースソート
```

**カスタム比較でソート**

```cpp
// スコアの降順でソート（projection使用）
ranges::sort(items_with_scores, ranges::greater{},
    [](const auto & p) { return p.second; });
```

**actions::remove_if**

```cpp
#include <range/v3/action/remove_if.hpp>

std::vector<Item> items = ...;
items |= ranges::actions::remove_if([](const auto & item) {
    return !item.valid();
});
```

---

## ビュー版/ベクトル版の2層実装パターン

### 柔軟なAPI設計

呼び出し側で実体化の有無を選べるようにする設計パターン：

```cpp
class Container {
    std::vector<std::shared_ptr<Item>> items_;

public:
    // ビュー版：遅延評価、メモリ効率が良い
    [[nodiscard]] auto getAvailableItemsView(int exclude_id = -1) const
        -> decltype(auto)
    {
        return items_ | ranges::views::filter([exclude_id](const auto & item) {
            return item->available() && item->id != exclude_id;
        });
    }

    // ベクトル版：即座に実体化
    [[nodiscard]] auto getAvailableItems(int exclude_id = -1) const
        -> std::vector<std::shared_ptr<Item>>
    {
        return getAvailableItemsView(exclude_id) | ranges::to<std::vector>();
    }

    // ID一覧を取得
    [[nodiscard]] auto getAvailableItemIds(int exclude_id = -1) const
        -> std::vector<int>
    {
        return getAvailableItemsView(exclude_id)
            | ranges::views::transform([](const auto & item) { return item->id; })
            | ranges::to<std::vector>();
    }
};
```

**使い分け**

```cpp
Container container;

// 1回だけ走査 → ビュー版
for (const auto & item : container.getAvailableItemsView()) {
    process(item);
}

// 複数回使う・サイズが必要 → ベクトル版
auto items = container.getAvailableItems();
std::cout << "Count: " << items.size() << std::endl;
```

---

## ベストプラクティス

### DO（推奨）

**空チェックを忘れない**

```cpp
auto view = data | ranges::views::filter(...);
auto result = ranges::empty(view) ? default_val : ranges::min(view);
```

**複雑なロジックはlambda内に記述**

```cpp
ranges::views::filter([&](const auto & item) {
    // 複数条件を1つのlambdaにまとめる
    return item.distance() >= min_dist &&
           item.angle() <= max_angle &&
           item.available();
})
```

**構造化バインディングを活用**

```cpp
for (const auto & [index, item] : ranges::views::enumerate(items)) { }
for (const auto & [item, score] : items_with_scores) { }
```

### DON'T（非推奨）

**ビューの寿命に注意**

```cpp
// 危険：元データが消えるとビューも無効
auto get_view() {
    std::vector<int> local_data = {1, 2, 3};
    return local_data | ranges::views::filter(...);  // ダングリング！
}

// 安全：実体化して返す
auto get_data() {
    std::vector<int> local_data = {1, 2, 3};
    return local_data | ranges::views::filter(...) | ranges::to<std::vector>();
}
```

**不必要な中間コンテナを避ける**

```cpp
// 悪い例
auto v1 = data | ranges::views::filter(...) | ranges::to<std::vector>();
auto v2 = v1 | ranges::views::transform(...) | ranges::to<std::vector>();

// 良い例：パイプラインを1つにまとめる
auto result = data
    | ranges::views::filter(...)
    | ranges::views::transform(...)
    | ranges::to<std::vector>();
```

---

## クイックリファレンス

| やりたいこと | 使うもの |
|-------------|---------|
| 条件でフィルタ | `views::filter` |
| 値を変換 | `views::transform` |
| インデックス付きループ | `views::enumerate` |
| 数列生成 | `views::iota` |
| 最小/最大値 | `min` / `max` |
| 最小/最大要素 | `min_element` / `max_element` |
| 条件検索 | `find_if` |
| 存在確認 | `contains` |
| vectorに変換 | `to<std::vector>()` |
| インプレースソート | `actions::sort` |
| 空チェック | `ranges::empty()` |
| 先頭N個スキップ | `views::drop` |
| 先頭N個だけ | `views::take` |

---

## 参考リンク

- [range-v3 公式ドキュメント](https://ericniebler.github.io/range-v3/)
- [range-v3 GitHub](https://github.com/ericniebler/range-v3)
- [C++20 Ranges（標準ライブラリ）](https://en.cppreference.com/w/cpp/ranges)
