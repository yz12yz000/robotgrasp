项目默认使用上一级目录中已提交的模型：

```text
../yolo26n-seg_openvino_model/
  *.xml
  *.bin
  metadata.yaml
```

构建时该模型目录会安装到 package share。也可以在 JSON 中将
`model_path` 指向其他外部模型目录；相对路径以已安装的 package share 为基准。
确保模型导出输入尺寸与配置 `imgsz` 匹配，并保留类别元数据。
