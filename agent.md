# hithesis（中文硕士）使用说明

## 1. 总体结构
- 主控文件：`examples/hitbook/chinese/thesis.tex`
- 正文章节：`examples/hitbook/chinese/body/*.tex`
- 后置章节（如结论）：`examples/hitbook/chinese/back/*.tex`

`thesis.tex` 主要用于组织结构，不建议把长篇正文直接写在这里。

## 2. 章节组织（当前配置）
在 `examples/hitbook/chinese/thesis.tex` 中当前顺序为：
- `\\include{body/regu}`
- `\\include{body/introduction}`
- `\\include{body/recruit}`
- `\\backmatter`
- `\\include{back/conclusion}`

对应写作文件：
- `examples/hitbook/chinese/body/regu.tex`
- `examples/hitbook/chinese/body/introduction.tex`
- `examples/hitbook/chinese/body/recruit.tex`
- `examples/hitbook/chinese/back/conclusion.tex`

## 3. 如何新增/调整章节
1. 在 `examples/hitbook/chinese/body/` 新建章节文件，例如：`method.tex`
2. 在 `thesis.tex` 的 `\\mainmatter` 区域添加：`\\include{body/method}`
3. 通过调整 `\\include{...}` 的顺序控制章节顺序

## 4. 编译方式（Windows PowerShell）
在目录 `examples/hitbook/chinese` 下执行：

```powershell
latexmk
```

清理中间文件：

```powershell
latexmk -c
```

## 5. 文献位置
- 文献库：`examples/hitbook/chinese/reference.bib`
- 参考文献样式与输出由 `thesis.tex` 中这两行控制：
  - `\\bibliographystyle{hithesis}`
  - `\\bibliography{reference}`

## 6. 常改参数（在 `thesis.tex`）
```tex
\\documentclass[fontset=fandol,type=master,campus=harbin]{hithesisbook}
```
常用项：
- `type=master`（硕士）
- `campus=harbin|shenzhen|weihai`
- `fontset=...`（按系统字体环境调整）
