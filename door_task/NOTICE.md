# 模型来源

`assets/lever_door.xml` 改编自 robosuite 1.5.2 的
[`robosuite/models/assets/objects/door_lock.xml`](https://github.com/ARISE-Initiative/robosuite/blob/v1.5.2/robosuite/models/assets/objects/door_lock.xml)。
保留其门铰链、杠杆把手、旋转锁舌及门框的机械关系；修改配色、摩擦、阻尼和门的角度上限，增加门框横梁，移除外部纹理依赖。
上游 XML 注明机构源自 [mj_envs](https://github.com/vikashplus/mj_envs/tree/master/mj_envs/hand_manipulation_suite/assets)。

`environment.py` 继承 robosuite 的 `Door`，场景组合方式参考该版本的 `Door._load_model()`。
robosuite 的许可及随附声明保存在 [LICENSE.robosuite](LICENSE.robosuite)。

这是用于任务开发的简化台架模型，尺寸、惯量与摩擦参数尚未根据真实门锁标定。
旋转锁舌绕开门框后允许开门，不包含住宅门锁的滑动锁舌传动机构。
