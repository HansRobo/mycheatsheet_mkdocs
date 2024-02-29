---
created: 2024-02-29 10:41:27
---

## パラメータ

### Nodeからいじる

#### get_parameter

```C++
// その1
bool bool_param = node.get_parameter("bool_param").as_bool();
// その2
rclcpp::Parameter parameter;
if(node.get_parameter("any_type_param", parameter)){
	std::cout << "get any type parameter" << std::endl;
}
// その3
if(get_parameter<bool>("bool_param", bool_param)){
	std::cout << "get bool parameter: " << bool_param << std::endl;
}
```

##### その1

```
rclcpp::Parameter get_parameter(const std::string & name) const
```

名前を指定して、パラメータを返す。  
存在しないパラメータに対しては `rclcpp::exceptions::ParameterNotDeclaredException`を送出する。

##### その2
```
bool get_parameter(const std::string & name, rclcpp::Parameter & parameter) const
```

名前を指定して、引数のパラメータ参照に代入する。  
存在しないパラメータの場合、falseが返る。  

##### その3

```
template<typename ParameterT>
bool get_parameter(const std::string & name, ParameterT & parameter) const
```

2のパラメータ型をテンプレートで指定するバージョン。  
名前を指定して、引数の生パラメータ型の参照に代入する。  
存在しないパラメータの場合、falseが返る。  
テンプレートで指定したパラメータ型と取得しようとしているパラメータの型が異なる場合、`rclcpp::ParameterTypeException`を送出する。  

#### get_parameter_or
##### その1
```
template<typename ParameterT>
bool
get_parameter_or(
  const std::string & name,
  ParameterT & parameter,
  const ParameterT & alternative_value) const
```

`get_parameter`その3のラッパーで、3の返り値がfalseのときに3番目の引数を返す。  
**パラメータが宣言されていないときに自動で宣言してくれるような機能はない。**  
返り値はROS空間からパラメータを取得できたかどうか（=alternative_valueが使われたときはfalseが返る）  
他のget_parameterと違って実装はnode_impl.hppにあるので注意。  

##### その2

```
template<typename ParameterT>
ParameterT
get_parameter_or(
  const std::string & name,
  const ParameterT & alternative_value) const
```

4のラッパーになっていて、パラメータを参照ではなく返り値で返してくれる。

