# Modern C++

## 常量

### nullptr

替代 `NULL` 。C++11 引入了 `nullptr` 关键字，专门用来区分空指针、0 。而 nullptr 的类型为 nullptr_t ，能
够隐式的转换为任何指针或成员指针的类型。

```cpp
foo(0);       // 调用 foo(int)
// foo(NULL); // 不能通过编译
foo(nullptr); // 调用 foo(char*)
```

### constexpr

显式声明常量表达式。 `const` 修饰的变量只在一定情况下是常量表达式，这有时可能带来困扰。C++11 提供了  `constexpr` 让用户显式的声明函数或对象构造函数在 **编译期** 会成为常量表达式。从 C++14 开始，constexpr 函数可以在内部使用局部变量、循环和分支等简单语句。

```cpp
const int len_1 = 5; // 常量表达式
const int len_2 = len_1 + 1; // 常量表达式
constexpr int len_2_constexpr = 1 + 2 + 3; // 显式声明的常量表达式
int len = 5;
const int len_3 = len + 1; // 非常量表达式
// 使用static_assert，当其第一个参数为常量表达式时才不会报错
static_assert(len_1, "");
static_assert(len_2, "");
static_assert(len_2_constexpr, "");
static_assert(len_3, "");  // 报错，说明len_3不是常量表达式
```

## 类型推导

### auto

从 C++11 起, 可以使用 `auto` 关键字进行类型推导。

```cpp
class MagicFoo {
public:
    std::vector<int> vec;
    MagicFoo(std::initializer_list<int> list) {
        // 不用再写冗长的迭代器类型名了
        for (auto it = list.begin(); it != list.end(); ++it) {
            vec.push_back(*it);
        }
    }
};

auto i = 5; // i 被推导为 int
auto arr = new auto(10); // arr 被推导为 int 
```

从 C++20 起， `auto` 甚至能用于函数传参。

```cpp
int add(auto x, auto y) {
    return x+y;
}
```

## decltype

`decltype` 关键字是为了解决 `auto` 关键字只能对变量进行类型推导的缺陷而出现的，可推导表达式的类型。

```cpp
auto x = 1;
auto y = 2;
decltype(x+y) z; // z的类型是int
```

`std::is_same<T, U>` 用于判断 T 和 U 这两个类型是否相等。

```cpp
if (std::is_same<decltype(x), int>::value)   // 为真
    std::cout << "type x == int" << std::endl;
if (std::is_same<decltype(x), float>::value) // 为假
    std::cout << "type x == float" << std::endl;
```

### 尾返回类型推导

C++11引入了一个尾返回类型（trailing return type），利用 `auto` 关键字将返回类型后置。C++14开始可以直接让普通函数具备返回值推导。

```cpp
// after c++11
template<typename T, typename U>
auto add2(T x, U y) -> decltype(x+y){
    return x + y;
}
// after c++14
template<typename T, typename U>
auto add3(T x, U y){
    return x + y;
}
```

### decltype(auto)

C++14引入的 `decltype(auto)` 主要用于对转发函数或封装的返回类型进行推导，它使我们无需显式指定 decltype 的参数表达式。

```cpp
tsd::string lookup1();
std::string& lookup2();

// C++11
std::string look_up_a_string_1() {
    return lookup1();
}
std::string& look_up_a_string_2() {
    return lookup2();
}

// after C++14
decltype(auto) look_up_a_string_1() {
    return lookup1();
}
decltype(auto) look_up_a_string_2() {
    return lookup2();
}
```

## 控制流

### 基于范围的for循环

C++11 引入了基于范围的循环写法。

```cpp
std::vector<int> vec = {1, 2, 3, 4};
for (auto element : vec)
    std::cout << element << std::endl; // 不会改变vec的元素，用于读
for (auto &element : vec)
    element += 1;         // 可以改变vec的元素，用于写
```

## 面向对象

### 委托构造

C++11 引入了委托构造的概念，这使得构造函数可以在同一个类中一个构造函数调用另一个构造函数。

```cpp
#include <iostream>

class Base {
public:
    int value1;
    int value2;
    Base() {
        value1 = 1;
    }
    Base(int value) : Base() { // 委托 Base() 构造函数
        value2 = value;
    }
};

Base b(2);
```

### 继承构造

C++11 利用关键字 `using` 引入了继承构造函数。

```cpp
#include <iostream>

class Base {
public:
    int value1;
    int value2;
    Base() {
        value1 = 1;
    }
    Base(int value) : Base() { // 委托 Base() 构造函数
        value2 = value;
    }
};

class Subclass : public Base {
public:
    using Base::Base; // 继承构造
};
```

### 显式虚函数重载

C++11 引入了 `override` 和 `final` 这两个关键字来防止 **意外重载虚函数** 和 **基类的虚函数被删除后子类的对应函数变为普通方法** 的情况发生。

#### override

当重载虚函数时，引入 `override` 关键字将显式的告知编译器进行重载，编译器将检查基类是否存在这样的虚函数，否则将无法通过编译：

```cpp
struct Base {
    virtual void foo(int); 
};

struct SubClass: Base {
    virtual void foo(int) override;   // 合法
    virtual void foo(float) override; // 非法, 父类没有此虚函数
};
```

#### final

`final` 则是为了防止类被继续继承以及终止虚函数继续重载引入的。

```cpp
struct Base {
    virtual void foo() final;
};

struct SubClass1 final: Base {
}; // 合法

struct SubClass2 : SubClass1 {
}; // 非法, SubClass1 已 final

struct SubClass3: Base {
    void foo(); // 非法, foo 已 final
};
```

### 显式禁用默认函数

C++11允许显式的声明采用或拒绝编译器默认生成的函数。

```cpp
class Magic {
public:
    Magic() = default; // 显式声明使用编译器生成的构造
    Magic& operator=(const Magic&) = delete; // 显式声明拒绝编译器生成默认赋值函数
    Magic(int magic_number);
}
```

### 强类型枚举

C++11引入了枚举类（enumeration class），并使用 `enum class` 的语法进行声明。枚举类实现了类型安全，首先他不能够被隐式的转换为整数，同时也不能够将其与整数数字进行比较， 更不可能对不同的枚举类型的枚举值进行比较。希望获得枚举值的值时，必须显式的进行类型转换。

```cpp
#include <iostream>

enum class new_enum : unsigned int {
    value1,
    value2,
    value3 = 100,
    value4 = 100
};

int main(){
    if (new_enum::value3 == new_enum::value4) {
        std::cout << "new_enum::value3 == new_enum::value4" << std::endl;
    }
    std::cout<<(int)new_enum::value4;
    return 0;
}
```

## Lambda 表达式

Lambda 表达式是现代 C++ 中最重要的特性之一，而 Lambda 表达式，实际上就是提供了一个类似匿名函数的特性，而匿名函数则是在需要一个函数，但是又不想费力去命名一个函数的情况下去使用的。

### 基础

Lambda 表达式的基本语法如下：

```cpp
[捕获列表](参数列表) 异常标识(可选) 属性标识(可选) -> 返回类型 {
// 函数体
}
```

所谓捕获列表，其实可以理解为一种参数，Lambda 表达式内部函数体在默认情况下是不能够使用函数体外部的变量的，这时捕获列表可以起到传递外部数据的作用。根据传递的行为，变量捕获也分为以下几种。

### 不捕获

如果是空的方括号，表示不捕获任何变量。

```cpp
auto f = [](int a, int b) {
    return a+b;
}
```

如果变量名前面有引用符号 **&** ，则是按引用捕获，可以修改外部变量的值；

如果不加 **&** ，就是按值捕获，不可以修改外部变量的值。

### 按值捕获

与参数传值类似，按值捕获的前提是变量可以拷贝，不同之处则在于，被捕获的变量在 Lambda 表达式被创建时拷贝，而非调用时才拷贝。

```cpp
int value = 1;
auto copy_value = [value] {
    return value;
};
value = 100;
auto stored_value = copy_value();
// stored_value == 1, 而 value == 100.
// 因为 copy_value 在创建时就保存了一份 value 的拷贝
```

### 按引用捕获

与引用传参类似，引用捕获保存的是引用，值会发生变化。

```cpp
int value = 1;
auto copy_value = [&value] {
    return value;
};
value = 100;
auto stored_value = copy_value();
// 这时, stored_value == 100, value == 100.
// 因为 copy_value 保存的是引用
```

### 隐式捕获

如果方括号内只写 **&** ，则按引用捕获所有外部变量；如果方括号内只写 **=** ，则按值捕获所有外部变量。

还有一种写法，可以指定一些变量按值捕获，其他变量都按引用捕获。例如 `[&, =N]` 按值捕获N，其他变量按引用捕获。

所以这几个捕获语句的效果是等价的，都是按引用捕获N，按值捕获M。
```cpp
[&N, M]  [M, &N]  [&, =M]  [=, &M]
```

### 泛型Lambda

从 C++14 开始， Lambda 函数的形式参数可以使用 `auto` 关键字来自动推导参数类型。

```cpp
auto add = [](auto x, auto y) {
    return x+y;
};

add(1, 2);
add(1.1, 2.2);
```

## 智能指针与内存管理

### RAII 与引用计数

引用计数是为了防止内存泄露而产生的。基本想法是对于动态分配的对象，进行引用计数，每当增加一次对同一个对象的引用，那么引用对象的引用计数就会增加一次，每删除一次引用，引用计数就会减一，当一个对象的引用计数减为零时，就自动删除指向的堆内存。

在传统 C++ 中，『记得』手动释放资源，总不是最佳实践。因为我们很有可能就忘记了去释放资源而导致泄露。 所以通常的做法是对于一个对象而言，我们在构造函数的时候申请空间，而在析构函数（在离开作用域时调用）的时候释放空间， 也就是 RAII 资源获取即初始化技术。

C++11 引入智能指针的概念，让程序员不再需要关心手动释放内存。使用它们需要包含头文件 `<memory>` 。

### std::unique_ptr

`std::unique_ptr` 是一种独占的智能指针，它禁止其他智能指针与其共享同一个对象，从而保证代码的安全。

```cpp
std::unique_ptr<int> pointer = std::make_unique<int>(10); // make_unique 从 C++14 引入
std::unique_ptr<int> pointer2 = pointer; // 非法
```

既然是独占，换句话说就是不可复制。但是，我们可以利用 `std::move` 将其转移给其他的 unique_ptr 。

```cpp
#include <memory>

struct Foo {
    Foo() { std::cout << "Foo::Foo" << std::endl; }
    ~Foo() { std::cout << "Foo::~Foo" << std::endl; }
    void foo() { std::cout << "Foo::foo" << std::endl; }
};

void f(const Foo &) {
    std::cout << "f(const Foo&)" << std::endl;
}

int main() {
    std::unique_ptr<Foo> p1(std::make_unique<Foo>());
    // p1 不空, 输出
    if (p1) p1->foo();
    {
        std::unique_ptr<Foo> p2(std::move(p1));
        // p2 不空, 输出
        f(*p2);
        // p2 不空, 输出
        if(p2) p2->foo();
        // p1 为空, 无输出
        if(p1) p1->foo();
        p1 = std::move(p2);
        // p2 为空, 无输出
        if(p2) p2->foo();
        std::cout << "p2 被销毁" << std::endl;
    }
    // p1 不空, 输出
    if (p1) p1->foo();
    // Foo 的实例会在离开作用域时被销毁
}
```

此外，由于独占， `std::unique_ptr` 不会有引用计数的开销。

### std::shared_ptr

`std::shared_ptr` 是一种智能指针，它能够记录多个 shared_ptr 共同指向一个对象，从而消除显式的调用 `delete` ，当引用计数变为零的时候就会将对象自动删除。

但还不够，因为使用 `std::shared_ptr` 仍然需要使用 `new` 来调用，这使得代码出现了某种程度上的不对称。 `std::make_shared` 就能够用来消除显式的使用 `new` ，会分配创建传入参数中的对象，并返回这个对象类型的 `std::shared_ptr` 指针。

```cpp
#include <iostream>
#include <memory>

void foo(std::shared_ptr<int> i) {
    (*i)++;
}

int main() {
    // Constructed a std::shared_ptr
    auto pointer = std::make_shared<int>(10);
    foo(pointer);
    std::cout << *pointer << std::endl; // 11
    // The shared_ptr will be destructed before leaving the scope
    return 0;
}
```

## 模板

模板是泛型编程的基础，是编程中的蓝图，它将算法的思想给拓印下来，以一种独立于任何特定类型的方式编写代码。

### 函数模板

模板函数定义的一般形式如下：

```cpp
template <typename type> ret-type func-name(parameter list)
{
   // 函数体
}
```

下面是函数模板的实例，返回两个数中的最大值：

```cpp
#include <iostream>
#include <string>
 
using namespace std;
 
template <typename T>
inline T const& Max (T const& a, T const& b) 
{ 
    return a < b ? b:a; 
}

int main ()
{
 
    int i = 39;
    int j = 20;
    cout << "Max(i, j): " << Max(i, j) << endl;    //39
 
    double f1 = 13.5; 
    double f2 = 20.7; 
    cout << "Max(f1, f2): " << Max(f1, f2) << endl;  //20.7

    return 0;
}
```
