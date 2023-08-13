# Protobuf&Yaml

## 1.数据的传输与解析——序列化与反序列化

在网络通信的过程中，常常需要进行对象的传输。对象中常常含有不同的变量：

- 整数
- 字符串
- 数组
- 数组对象
- ...

那么我们如何正确地进行这种传递呢？要想实现对象的传输，在发送端我们需要使用一定的规则，将对象转换为具体的字节数组，这就是 **序列化(serialization)** ；而在接收端再以这种规则将字节数组还原为对象，这就是 **反序列化(deserialization)** 。

常见的序列化-反序列化协议有 [XML](https://www.w3.org/TR/xml/)、[JSON](https://www.json.org/json-zh.html)、[Protobuf](https://protobuf.dev/)、[YAML](https://yaml.org/)。

- JSON(JavaScript Object Notation，JavaScript对象简谱) 使用JavaScript构造对象的方法来存储、传输数据。
- XML(eXtensible Markup Language，可扩展标记语言) 使用标签 `<xx>` 和 `</xx>` 来区隔不同的数据。
- YAML(YAML Ain't a Markup Language) 是一个可读性高，用来表达数据序列化的格式。
- Protobuf(Protocol Buffers) 是Google公司开源跨平台的序列化数据结构的协议。

我们通过一个实例说明它们的差异。我们不妨定义以下对象：

```cpp
#include <string>
class Helloworld
{
    int id;
    std::string name;
    }
int main()
{
    Helloworld helloworld(101, "hello");
}
```

使用JSON序列化该对象：

```json
{
"id": 101,
"name": "hello"
}
```

使用XML序列化该对象：

```xml
<helloworld>
    <id>101</id>
    <name>hello</name>
</helloworld>
```

使用YAML序列化该对象：

```yaml
id: 101
name: "hello"
```

使用Protobuf序列化该对象（16进制格式）：

```protobuf
08 65 12 06 48 65 6C 6C 6F 77
```

根据上述实例，我们可以用一张表格总结四者的差异：

|               | JSON | XML | YAML | Protobuf |
|---------------|------|-----|------|----------|
|数据存储格式    |文本  |文本  |文本  | 二进制    |
|可读性          | 好   |较好 |较好   | 差       |
|存储空间        | 较大 |大   |大     | 小       |
|序列化/反序列速度| 慢   |慢   |慢    | 快       |

> 本节我们将重点介绍Protobuf与YAML的使用方法。但XML及其各种变体（如HTML）和JSON也在程序开发中有着广泛应用。感兴趣的同学可以参考相关资料了解[XML](https://www.w3school.com.cn/xml/xml_intro.asp)和[JSON](https://www.w3school.com.cn/js/js_json_intro.asp)的更多使用方法。


## 2.Protobuf

### 2-1.protobuf的安装

参考自[Protobuf C++ Installation](https://github.com/protocolbuffers/protobuf/blob/main/src/README.md)

```bash
sudo apt-get install autoconf automake libtool curl make g++ unzip
# 安装所需要的工具包
git clone https://github.com/protocolbuffers/protobuf.git
# 若网络不佳，可以将指令换为 git clone
https://gitee.com/mirrors/protobuf_source.git ./protobuf
cd protobuf
# (optional) git submodule update --init --recursive
git checkout 3.20.x # 根据版本需求选择不同的分支
./autogen.sh
./configure
make -j$(nproc)
sudo make install
sudo ldconfig
```

以上操作会将 protoc 可执行文件以及与 protobuf 相关的头文件、库安装至本机。在终端输入 `protoc` ，若输出提示信息，则表示安装成功。

### 2-2.proto文件

在使用 protobuf 时，我们首先需要在 **.proto** 文件中将需要被序列化的数据结构进行定义。

一个 proto 文件示例如下：

```protobuf
// import "other_protos.proto"; // 如果需要引用其它的protobuf文件，可以使用import语句。

syntax = "proto3"; // 指定protobuf遵循的语法格式是proto2还是proto3。在本教程和之后的开发中，我们都使用proto3语法格式。
package student; // 包名声明。如在本例中，proto文件生成的类都会被放在namespace student中，这一举措的意义在于防止命名冲突

enum Sex // 自定义枚举类型
{
    MALE = 0;
    FEMALE = 1;
}

message Course // protobuf中，使用message定义数据结构，类似于C中的结构体
{
    int32 credit = 1;
    string name = 2;
}

message StudentInfo
{
    // 变量声明格式 <限定修饰符> <数据类型> <变量名>=id
    int32 age = 1;
    string name = 2;
    Sex sex = 3;
    repeated Course courses = 4; // repeated表示重复（数组），本例也表明message可以嵌套message
}
```

**protobuf语法标准**

protobuf有两套语法标准：proto2和proto3，两套语法不完全兼容。我们可以使用 syntax 关键字指定protobuf遵循的语法标准。

**编号**

消息定义中的每个字段都有一个唯一的编号，从1开始。这些字段号用于识别你在二进制格式消息中的信息。

一个常见的约定是，我们会将经常使用的字段编号为1-15，不常用的字段编号为16以上的数字，因为1-15的编号编码仅需要1 byte，这样可以减小字节流的体积。

**数据类型**

| proto Type | C++ Type | Python Type | C# Type |
|------------|----------|-------------|---------|
|double      | double   | float       | double  |
|float       | float    | float       | float   |
|int32       | int32    | int         | int     |
|int64       | int64    | int/long    | long    |
|uint32      | uint32   | int/long    | uint    |
|uint64      | uint64   | int/long    | ulong   |
|sint32      | int32    | int         | int     |
|sint64      | int64    | int/long    | long    |
|fixed32     | uint32   | int/long    | uint    |
|fixed64     | uint64   | int/long    | ulong   |
|sfixed32    | int32    | int         | int     |
|sfixed64    | int64    | int/long    | long    |
|bool        | bool     | bool        | bool    |
|string      | string   | str/unicode | string  |
|bytes       | string   | str(Python2) bytes(Python 3)| ByteString|

更多语言的对应关系参看 [Protobuf scalar types](https://developers.google.com/protocol-buffers/docs/proto3#scalar)。

此外，Protobuf还支持使用 enum 关键字定义枚举类型。每个枚举定义都必须包含一个映射到0的常量作为枚举的默认值。

为了尽可能多地压缩数据，Protobuf对各数据类型地默认值做了以下处理：

- numeric types : 0
- bool : false
- string : 空字符串
- byte : 空字节
- enum : 第一个定义的枚举值（0）
- message : 取决于目标编程语言

**repeated**

repeated 关键字可以定义重复多次的信息（即数组），其顺序是有序的。

**命名法**

为了便于阅读，protobuf规定了一系列命名法：

- message、enum采用大驼峰命名法，如 message StudentInfo 。
- 字段采用下划线分割法，且全部小写，如 string student_name 。
- 枚举值采用下划线分割法，且全部大写，如 FIRST_VALUE 。

### 2-3.使用proto文件进行序列化和反序列化

**生成目标语言文件**

编写好的proto文件不能直接应用于工程中，我们需要使用 protoc 工具生成对应的文件（以C++为例）：

```bash
protoc --help # 查看使用方法
protoc test.proto --cpp_out=. # 在当前目录下生成.cpp文件和.h文件
```

使用 `--cpp_out` 选项，会生成 `<protobuf_name>.pb.h` 文件和 `<protobuf_name>.pb.cc` 文件。生成的文件中会将proto文件中定义的message转换为对应的类，供C++程序使用。

**在C++程序中使用protobuf工具的例程如下：**

```cpp
#include <iostream>
#include <fstream>
#include <vector>
#include <google/protobuf/message.h> // for protobuf
#include "test.pb.h" // for protobuf source file

int main()
{
    // 可以看到，protobuf文件中的信息都被封装在namespace student中，这是之前protobuf中的`package`语法所规定的。

    // 1. 如何实例化一个proto文件中定义的类
    student::StudentInfo student1;

    // 2. 如何设置类的各个属性

    // a. 添加单一字段：使用set_<xxx>()语句
    student1.set_age(18);
    student1.set_name("Alice");
    student1.set_sex(student::Sex::female);

    // b. 添加repeated字段：使用add_<xxx>()语句
    student::Course* course1 = student1.add_courses();
    course1 -> set_name("calculus");
    course1 -> set_credit(5);
    student::Course* course2 = student1.add_courses();
    course2 -> set_name("Fundamentals of Electronic Circuits and System");
    course2 -> set_credit(2);

    // 3. 如何使用类的各个属性：使用<xxx>()语句
    std::cout << "----------------student info----------------" << std::endl
    << "age: " << student1.age() << std::endl
    << "name: " << student1.name() << std::endl
    << "sex (0:male, 1:female): " << (int)student1.sex() << std::endl
    << "courses: " << std::endl;
    for(int i = 0;i<student1.courses_size();i++)
    {
        std::cout << " " << i << ". "
        << "name: " << student1.courses(i).name() << " "
        << "credit: " << student1.courses(i).credit() << std::endl;
    }
    std::cout << "--------------------------------------------" << std::endl;

    // 4. 序列化
    std::cout << "serialize to file." << std::endl;
    std::fstream output("./output", std::ios::out | std::ios::binary );
    student1.SerializeToOstream(&output); // 序列化为流

    std::cout << "serialize to array." << std::endl;
    size_t size = student1.ByteSizeLong();
    unsigned char* data = new unsigned char [size];
    student1.SerializeToArray(data, student1.ByteSizeLong()); // 序列化为数组

    // 5. 反序列化和debug
    std::cout << "deserialize from array." << std::endl;
    student::StudentInfo studentInfoFromArray;
    std::cout << std::endl;
    studentInfoFromArray.ParseFromArray(data, size);
    std::cout << studentInfoFromArray.DebugString() << std::endl; // 输出字符串化的信息
}
```

需要指出的是，想要成功生成可执行文件，需要链接protobuf的静态库和动态库。在Linux系统上应用使用到protobuf的C++工程，最好的方法是使用CMake。在本例中，库的依赖关系由CMake工具处理。

### 2-4.prototxt文件

在RoboRTS-icra2019中可以看到其生成的结构以及参数，并不是二进制形式，而是prototxt形式。该形式是ProtoBuf数据保存的另外一种可视化形式，在需要经常修改配置、调参的情况下，可以采用这种形式，该形式的生成格式是与proto中的数据结构相对应的。

**以decision包为例如下：**

```protobuf
# roborts_decision/proto/decision.proto

syntax = "proto2";

package roborts_decision;

message Point {
    optional float x = 1;
    optional float y = 2;
    optional float z = 3;

    optional float roll  = 4;
    optional float pitch = 5;
    optional float yaw   = 6;
}

message DecisionConfig {
    repeated Point point = 1;
    optional bool simulate = 2 [default = false];
    optional bool master = 3 [default = false];
    repeated Point escape = 4;
    repeated Point buff_point = 5;
    repeated Point search_path = 6;
    repeated Point master_bot = 7;
    repeated Point wait_point = 8;
}
```

```protobuf
# roborts_decision/config/decision.prototxt

simulate: true
master: true

master_bot {
    x:7.8
    y:0.6
    z: 0

    roll:  0
    pitch: 0
    yaw:   3.14
}

master_bot {
   x:0.8
   y:4.2
   z: 0

   roll:  0
   pitch: 0
   yaw:   0
}
```

**读取prototxt文件**

```cpp
# roborts_common/include/io/io.h

template<class T>
inline bool ReadProtoFromTextFile(const char *file_name, T *proto) {
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  using google::protobuf::io::CodedInputStream;
  using google::protobuf::io::ZeroCopyOutputStream;
  using google::protobuf::io::CodedOutputStream;
  using google::protobuf::Message;

  std::string full_path = /*ros::package::getPath("roborts") +*/ std::string(file_name);
  ROS_INFO("Load prototxt: %s", full_path.c_str());

  int fd = open(full_path.c_str(), O_RDONLY);
  if (fd == -1) {
    ROS_ERROR("File not found: %s", full_path.c_str());
    return false;
  }
  FileInputStream *input = new FileInputStream(fd);
  bool success = google::protobuf::TextFormat::Parse(input, proto);
  delete input;
  close(fd);
  return success;
}

template<class T>
inline bool ReadProtoFromTextFile(const std::string &file_name, T *proto) {
  return ReadProtoFromTextFile(file_name.c_str(), proto);
}
```

```cpp
roborts_decision::DecisionConfig decision_config;
std::string proto_file_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

boot_position_.resize(decision_config.master_bot().size());
for (int i = 0; i != decision_config.master_bot().size(); i++)
{
    boot_position_[i].header.frame_id = "map";
    boot_position_[i].pose.position.x = decision_config.master_bot(i).x();
    boot_position_[i].pose.position.z = decision_config.master_bot(i).z();
    boot_position_[i].pose.position.y = decision_config.master_bot(i).y();
}
```

## 3.YAML

累了。大家自己去互联网学习一下吧。

[一文看懂 YAML](https://zhuanlan.zhihu.com/p/145173920)

## 4.最后

[Protobuf guides](https://protobuf.dev/programming-guides/)

[protobuf编码之varint/zigzag](https://izualzhy.cn/protobuf-encode-varint-and-zigzag) protobuf为什么可以获得如此高效的编码效果？这涉及到其底层算法——varint和zigzag算法。

[proto3与proto2的区别](https://blog.csdn.net/ymzhu385/article/details/122307593) proto3与proto2不兼容。

[为什么要序列化？](https://blog.csdn.net/wangyuan9826/article/details/123326167)