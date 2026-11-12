# 第二阶段:打通 ROS2

第一阶段的工具返回的是假数据。这一阶段让程序读到机器人的**真实状态**,再把它接回大模型。

**全部只读**:只订阅话题、查 TF、问 MoveIt“会不会撞”,不发任何控制指令,机器人不会动。让机器人动是第三阶段的内容。

## 能做什么

```text
你:左手末端现在在哪?
  [第0步] get_link_position({"link": "arm_l_end_link"}) -> {'xyz_m': [0.264, 0.9216, 1.352]}
大模型:左手末端在 base_link 下 x=0.264 m、y=0.922 m、z=1.352 m
```

## 文件和阅读顺序

按这个顺序读,每个文件只做一件事:

| 顺序 | 文件 | 做什么 | 需要 ROS 在跑吗 |
|---|---|---|---|
| 1 | `config.py` | 所有话题名、link 名、关节名、超时 | 否 |
| 2 | `ros_io.py` | **唯一碰 rclpy 的文件**:订阅关节和 URDF、查 TF、调 MoveIt | 是 |
| 3 | `kinematics.py` | 自己写的正运动学(FK),纯 numpy | 否 |
| 4 | `check_ros.py` | 5 步演示,把上面的东西逐个打印出来 | 是 |
| 5 | `tools.py` | 给大模型的 3 个只读工具 | 是 |
| 6 | `llm.py` + `agent.py` | 和第一阶段一样的循环,工具换成真的 | 是 |
| — | `fake_robot.py` | 没有真机时,发出和真机一样的话题 | — |

## 文件之间的关系

### 总览

```mermaid
flowchart LR
    agent["agent.py"] --> llm["llm.py"] --> deepseek(("DeepSeek API"))
    agent --> tools["tools.py"] --> rosio["ros_io.py"]
    check["check_ros.py"] --> rosio
    check --> kin["kinematics.py"]
    rosio <-.-> ros(("ROS2 网络"))
    fake["fake_robot.py"] -.-> ros
```

### 时序图

参与者的名字写成“文件名 / 函数或对象”，箭头上写的是**实际的函数调用和变量名**，和代码里一致。实线箭头表示调用，虚线箭头表示返回值或 ROS 消息。

#### ① 启动:`RosIO()` 建立连接,后台线程一直收数据

```mermaid
sequenceDiagram
    autonumber
    participant M as agent.py / check_ros.py<br/>main()
    participant R as ros_io.py<br/>RosIO.__init__()
    participant T as ros_io.py<br/>self._spin_thread(后台线程)
    participant N as ROS2 网络
    participant P as 真机 或 fake_robot.py<br/>+ robot_state_publisher

    M->>R: ros = RosIO("learning_agent")
    R->>R: rclpy.init()<br/>self.node = rclpy.create_node(node_name)
    R->>N: create_subscription(JointState, "/joint_states",<br/>self._on_joints, qos_profile_sensor_data)
    R->>N: create_subscription(String, "/robot_description",<br/>self._on_urdf, latched)
    R->>N: self.tf_buffer = Buffer()<br/>self.tf_listener = TransformListener(self.tf_buffer, self.node)
    R->>N: self.validity_client = create_client(GetStateValidity, "/check_state_validity")
    R->>T: self._spin_thread.start()  →  self._executor.spin()
    R-->>M: 返回 ros

    M->>R: ros.wait_ready()
    loop 每 0.05 秒看一次,最多 5 秒
        P-->>N: /robot_description(启动时只发一次,TRANSIENT_LOCAL)
        N-->>T: String 消息
        T->>T: _on_urdf(msg)<br/>with self._lock: self._urdf = msg.data
        P-->>N: /joint_states(50~几百 Hz,BEST_EFFORT)
        N-->>T: JointState 消息
        T->>T: _on_joints(msg)<br/>with self._lock:<br/>self._joints.update(zip(msg.name, msg.position))<br/>self._joints_time = time.monotonic()
        P-->>N: /tf(robot_state_publisher 算好的各 link 位姿)
        N-->>T: 存进 self.tf_buffer
    end
    R-->>M: self._joints 和 self._urdf 都有了 → return<br/>(超时则 raise RuntimeError,说明缺哪个话题)
```

后台线程一直在跑：`self._joints`、`self._urdf`、`self.tf_buffer` 始终是最新的，主线程随时读。

#### ② 一次问答:`agent.py` 问“左手末端在哪”

```mermaid
sequenceDiagram
    autonumber
    actor U as 你
    participant A as agent.py<br/>main() / run_agent()
    participant L as llm.py<br/>chat()
    participant D as DeepSeek API
    participant RT as agent.py<br/>run_tool()
    participant H as tools.py<br/>get_link_position()
    participant R as ros_io.py<br/>RosIO.lookup()
    participant B as self.tf_buffer<br/>(后台一直在更新)

    A->>A: key = llm.read_key()<br/>handlers = make_handlers(ros)
    U->>A: task = input("你: ")  → "左手末端在哪?"
    A->>A: messages = [{"role": "system", "content": SYSTEM},<br/>{"role": "user", "content": task}]

    A->>L: reply = llm.chat(key, messages, TOOLS)
    L->>D: POST body = {"model", "messages", "thinking", "tools": TOOLS}
    D-->>L: data["choices"][0]["message"]
    L-->>A: reply = {"content": "...", "tool_calls": [call]}
    A->>A: messages.append(reply)

    A->>RT: run_tool(handlers, name="get_link_position",<br/>raw_args='{"link": "arm_l_end_link"}')
    RT->>RT: func = handlers.get(name)<br/>args = json.loads(raw_args)
    RT->>H: func(**args)  →  get_link_position(link="arm_l_end_link")
    H->>R: xyz, _ = ros.lookup(config.BASE_FRAME, link)
    R->>B: tf = self.tf_buffer.lookup_transform("base_link",<br/>"arm_l_end_link", rclpy.time.Time(), timeout=1s)
    B-->>R: tf.transform.translation / rotation
    R-->>H: [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
    H-->>RT: {"ok": True, "link": link, "frame": "base_link", "xyz_m": [0.264, 0.9216, 1.352]}
    RT-->>A: result
    A->>A: messages.append({"role": "tool",<br/>"tool_call_id": call["id"],<br/>"content": json.dumps(result)})

    A->>L: reply = llm.chat(key, messages, TOOLS)  (第 2 轮)
    L->>D: POST(messages 里多了 tool_calls 和工具结果)
    D-->>L: 没有 tool_calls
    L-->>A: reply["content"] = "左手末端在 x=0.264 m ..."
    A-->>U: print("大模型:", answer)
```

#### ③ 碰撞检查:`check_joint_change` 问 MoveIt“肘再弯 90° 会不会撞”

```mermaid
sequenceDiagram
    autonumber
    participant RT as agent.py<br/>run_tool()
    participant H as tools.py<br/>check_joint_change()
    participant R as ros_io.py<br/>RosIO.check_collision()
    participant N as /check_state_validity
    participant MG as move_group(MoveIt)

    RT->>H: check_joint_change(group="left_arm",<br/>joint_index=4, delta_deg=90)
    H->>H: name = config.JOINT_GROUPS[group][joint_index - 1]<br/>→ "idx24_arm_l_joint4"
    H->>R: ros.joints([name])
    R-->>H: {name: 当前弧度}
    H->>H: new_value = 当前弧度 + math.radians(delta_deg)
    H->>R: res = ros.check_collision({name: new_value})

    alt self.validity_client.service_is_ready() 为 False
        R-->>H: {"available": False, "error": "/check_state_validity 不可用"}
    else 服务可用
        R->>R: state = self.joints()<br/>state.update(changes)
        R->>R: req = GetStateValidity.Request()<br/>req.robot_state.joint_state.name / position = state<br/>req.robot_state.is_diff = True<br/>req.group_name = ""(整台机器人)
        R->>N: future = self.validity_client.call_async(req)
        N->>MG: 用这组关节角做一次整机碰撞检测
        MG-->>N: res.valid, res.contacts
        N-->>R: future.done() → res = future.result()
        R-->>H: {"available": True, "valid": bool(res.valid),<br/>"contacts": [(contact_body_1, contact_body_2), ...]}
    end
    H-->>RT: {"ok": True, "joint": name, "collision": not valid, "contacts": [...]}
```

只是“假设”，`req` 里的关节角只发给 MoveIt 算，机器人不会动。

#### ④ `check_ros.py` 第 4 步:自己算 FK,和 TF 对比

```mermaid
sequenceDiagram
    autonumber
    participant C as check_ros.py<br/>step4_fk_vs_tf(ros, link)
    participant R as ros_io.py<br/>RosIO
    participant K as kinematics.py

    C->>R: ros.urdf_xml()
    R-->>C: self._urdf(XML 字符串)
    C->>K: robot = kinematics.load_urdf(xml)
    C->>R: joints = ros.joints()
    R-->>C: {关节名: 弧度}(30 个)
    C->>R: xyz, quat = ros.lookup(config.BASE_FRAME, link)
    R-->>C: TF 的答案

    C->>K: T = kinematics.fk(robot, "base_link", link, joints)
    K->>K: chain_joints(robot, base, tip) → 14 个 joint
    loop 链上每个 joint
        K->>K: joint_transform(joint, q)<br/>= make_transform(rpy_to_matrix(origin.rpy), origin.xyz)<br/>@ make_transform(R=axis_angle_matrix(axis, q))
        K->>K: T = T @ 这个 joint 的变换
    end
    K-->>C: T(4×4)

    C->>C: pos_err_mm = norm(T[:3, 3] - xyz) * 1000
    C->>K: rotation_diff_deg(T[:3, :3], quat_to_matrix(quat))
    K-->>C: rot_err(度)
    C->>C: print 两边的 xyz 和误差(静止时 ≈ 0)
```

#### ⑤ 假机器人:`fake_robot.py` 发出和真机一样的数据

```mermaid
sequenceDiagram
    autonumber
    participant F as fake_robot.py<br/>main()
    participant RSP as robot_state_publisher<br/>(子进程 rsp)
    participant N as ROS2 网络(domain 77)

    F->>F: refuse_robot_domain()<br/>ROS_DOMAIN_ID == "4" 或有 DEV_IP → sys.exit
    F->>F: urdf_xml = config.URDF_FILE.read_text()<br/>joint_names = 所有非 fixed 关节(30 个)
    F->>F: params_file:写入 {"robot_state_publisher":<br/>{"ros__parameters": {"robot_description": urdf_xml}}}
    F->>RSP: rsp = subprocess.Popen(["ros2", "run", "robot_state_publisher", ...,<br/>"--params-file", params_file.name])
    RSP->>N: /robot_description(TRANSIENT_LOCAL)
    F->>F: pub = node.create_publisher(JointState, "/joint_states", 10)
    loop 每 0.02 秒(50 Hz)
        F->>F: positions[left_arm 第 i 个] = 0.3 * sin(0.5 * t + i)<br/>其他关节 = 0.0
        F->>N: pub.publish(msg)  msg.name / msg.position
        N-->>RSP: /joint_states
        RSP->>N: /tf(用 URDF + 关节角算出每个 link 的位姿)
    end
    F->>RSP: Ctrl+C → rsp.terminate()<br/>os.remove(params_file.name)
```

看图时注意两点:

- **只有 `ros_io.py` 连到 ROS 网络。** 其他文件都通过它读数据。
- **TF 不是机器人直接发的。** `robot_state_publisher` 拿关节角和 URDF 算出来再发布。`kinematics.py` 做的是同一件事，所以两边结果一致。

只有 `ros_io.py` 知道 ROS 的存在。以后换成别的机器人,或者 ROS 接口变了,只改这一个文件。

## 运行

### 方式一:没有真机,用假机器人(推荐先用这个)

终端 1:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77          # 和真机(domain 4)隔开
cd ~/robot_harness
python3 learning/stage2_ros/fake_robot.py
```

终端 2(同样的两行环境):

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
cd ~/robot_harness
python3 learning/stage2_ros/check_ros.py
python3 learning/stage2_ros/agent.py
```

`fake_robot.py` 在 domain 4 或者 source 过 `env_g2.sh` 的终端里会拒绝运行。否则假的关节角会发到真机网络里,真机的 MoveIt 会把它当成真实数据。

假机器人没有 MoveIt,所以第 5 步会显示“不可用”,这是正常的。

### 方式二:真机

`hardware.launch.py` 由人在另一个终端启动,然后:

```bash
source ~/agi@g2_robot/env_g2.sh
cd ~/robot_harness
python3 learning/stage2_ros/check_ros.py
python3 learning/stage2_ros/agent.py
```

离线自测 FK(什么都不用开):

```bash
source /opt/ros/humble/setup.bash
python3 learning/stage2_ros/kinematics.py
```

## 这一阶段的 5 个关键概念

1. **订阅和回调**:`create_subscription(类型, 话题, 回调函数, QoS)`。每来一条消息,ROS 就调用一次回调。
2. **spin**:不 spin 就收不到任何消息。`RosIO` 在后台线程里一直 spin,主线程随时读最新数据。因为两个线程同时访问数据,所以要加锁。
3. **QoS 要和发布方匹配**:`/joint_states` 是 BEST_EFFORT,要用 `qos_profile_sensor_data`;`/robot_description` 只发一次,要用 `TRANSIENT_LOCAL`。不匹配不会报错,只是一条都收不到。
4. **TF 是怎么来的**:`robot_state_publisher` 用 URDF 和关节角做 FK,算出每个 link 的位置发到 `/tf`。`check_ros.py` 第 4 步自己算一遍,和 TF 对比,静止时差 0 mm。
5. **读 TF 不需要 MoveIt**:MoveIt 负责碰撞检查和规划。这里只用了它的碰撞检查服务,而且只是“假设这样转会不会撞”,机器人不动。

## 练习

1. `check_ros.py --link head_link3`:头在哪?
2. 在 `config.LINKS_FOR_LLM` 里加一个 link(比如 `arm_l_link2`),再问大模型它在哪。
3. 在 `fake_robot.py` 里让右臂也摆起来,看 `check_ros.py` 第 4 步的误差会不会变大,想一想为什么。
4. 给 `tools.py` 加一个工具 `get_distance(link_a, link_b)`:两个 link 之间的直线距离。现在大模型是自己心算的。
5. 真机上:先 `check_ros.py --step 5`,再问大模型“右臂第 2 关节再转 60 度会撞吗”,和第 5 步的结果对照。
