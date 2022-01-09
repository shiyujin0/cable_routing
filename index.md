## Robotic Cable Routing with Spatial Representation
Shiyu Jin, Wenzhao Lian, Changhao Wang, Masayoshi Tomizuka, and Stefan Schaal

Intrinsic Innovation LLC, UC Berkeley

### Abstract

Cable routing is a challenging task for robotic automation. To accomplish the task, it requires a high-level path planner to generate a sequence of cable configurations from the initial state to the target state and a low-level manipulation planner to plan the robot motion commands to transit between adjacent states. However, there are yet no proper representations to model the cable with the environment objects, impeding the design of both high-level path planning and low-level manipulation planning. In this paper, we propose a framework for cable routing with spatial representation. For high-level planning, by considering the spatial relations between the cable and the environment objects such as fixtures, the proposed method is able to plan a path from the initial state to the goal state in a graph. For low-level manipulation, multiple manipulation primitives are efficiently learned from human demonstration, to configure the cable to planned intermediate states leveraging the same spatial representation. We also implement a cable state estimator that robustly extracts the spatial representation from raw RGB-D images, thus completing the cable routing framework. We evaluate the proposed framework with various cables and fixture settings, and demonstrate that it outperforms some baselines in terms of reliability and generalizability.

### Proposed Framework

### Spatial Representation

### Learning Manipulation Primitives

### Segmentation Neural Network

### Experiments Videos (speed*10)


You can use the [editor on GitHub](https://github.com/shiyujin0/cable_routing/edit/gh-pages/index.md) to maintain and preview the content for your website in Markdown files.

Whenever you commit to this repository, GitHub Pages will run [Jekyll](https://jekyllrb.com/) to rebuild the pages in your site, from the content in your Markdown files.

### Markdown

Markdown is a lightweight and easy-to-use syntax for styling your writing. It includes conventions for

```markdown
Syntax highlighted code block

# Header 1
## Header 2
### Header 3

- Bulleted
- List

1. Numbered
2. List

**Bold** and _Italic_ and `Code` text

[Link](url) and ![Image](src)
```

For more details see [Basic writing and formatting syntax](https://docs.github.com/en/github/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax).

### Jekyll Themes

Your Pages site will use the layout and styles from the Jekyll theme you have selected in your [repository settings](https://github.com/shiyujin0/cable_routing/settings/pages). The name of this theme is saved in the Jekyll `_config.yml` configuration file.

### Support or Contact

Having trouble with Pages? Check out our [documentation](https://docs.github.com/categories/github-pages-basics/) or [contact support](https://support.github.com/contact) and we’ll help you sort it out.
