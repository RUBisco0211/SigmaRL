# A Learning-Based Control Barrier Function for Car-Like Robots: Toward Less Conservative Collision Avoidance

Jianye $\mathrm { { X u ^ { 1 } \mathbb { o } } }$ , Student Member, IEEE, Bassam Alrifaee2 , Senior Member, IEEE 

Abstract— We propose a learning-based Control Barrier Function (CBF) to reduce conservatism in collision avoidance for car-like robots. Traditional CBFs often use the Euclidean distance between robots’ centers as a safety margin, which neglects their headings and approximates their geometries as circles. Although this simplification meets the smoothness and differentiability requirements of CBFs, it may result in overly conservative behavior in dense environments. We address this by designing a safety margin that considers both the robot’s heading and actual shape, thereby enabling a more precise estimation of safe regions. Because this safety margin is nondifferentiable, we approximate it with a neural network to ensure differentiability. In addition, we propose a notion of relative dynamics that makes the learning process tractable. In a case study, we establish the theoretical foundation for applying this notion to a nonlinear kinematic bicycle model. Numerical experiments in overtaking and bypassing scenarios show that our approach reduces conservatism (e.g., requiring $3 3 . 5 \%$ less lateral space for bypassing) without incurring significant extra computation time. 

Code: github.com/bassamlab/sigmarl 

# I. INTRODUCTION

Collision avoidance for car-like robots often involves nonconvex optimization problems [1]. Control Barrier Functions (CBFs) provide a tool to replace non-convex safety constraints with affine constraints in the control input [2]. However, standard CBF formulations tend to over-approximate the actual geometry of car-like robots because they require a continuous and differentiable function [3]. A common simplification is to approximate the robots as circles, which simplifies distance computation but ignores the robots’ actual shapes and headings [4]–[14]. While this ensures the continuity and differentiability required by CBFs, it can lead to overly conservative behaviors, especially in dense environments. Figure 1 illustrates an example of conservatism caused by the circle approximation, where vehicle $i$ is prevented from overtaking vehicle $j$ . Our work aims to address this limitation by incorporating the actual geometries and headings of car-like robots into the safety constraints, thereby enabling less conservative collision avoidance. 

# A. Related Work

Collision avoidance for car-like robots is a wellstudied problem. Although optimization-based approaches 

This research was supported by the Bundesministerium fur Digitales und ¨ Verkehr (German Federal Ministry for Digital and Transport) within the project “Harmonizing Mobility” (grant number 19FS2035A). 

1Department of Computer Science, RWTH Aachen University, Germany, xu@embedded.rwth-aachen.de 

2Department of Aerospace Engineering, University of the Bundeswehr Munich, Germany, bassam.alrifaee@unibw.de 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/8960e0bac25fe91fdd9d65e0801ee0c8687c3eb9f6423488b191e6080fb62e50.jpg)



Fig. 1: An example of conservatism caused by the circle approximation. Vehicle $i$ is prevented from overtaking $j$ .


like model predictive control [15]–[18] are widely used, they can be computationally intensive. CBFs offer a way to enforce safety constraints in a computationally efficient manner [3]. 

Traditional CBFs for collision avoidance typically simplify the robot geometries as circles and define safety margins based on the Euclidean distance between the circles’ centers, referred to as the Center-to-Center (C2C)-based safety margin. In [4], CBFs were applied to ensure a safe distance from an avoidable set that establishes safe boundaries around round-shaped moving obstacles. In [5], safety barrier certificates were introduced for collision avoidance in multi-robot systems, where each robot is approximated as a circle. In [6], CBFs were extended to constraints with high relative degrees, validated using circular robots. In our previous work [7], a method called Truncated Taylor CBF was proposed to simplify control design for constraints with high relative degrees, with numerical experiments conducted on circular robots. In [8], off-center disks were used to reduce the conservatism inherent in the C2C-based safety margin. In the context of Connected and Automated Vehicles (CAVs), [9] and [10] applied a C2C-based safety margin within a multiagent Reinforcement Learning (RL) framework to ensure the safety of the learned policies. Other works employing the C2C-based safety margin include [11]–[14]. Although approximating robots as circles simplifies computations and maintains the differentiability required by CBFs, this simplification does not capture the actual shapes and orientations of car-like robots. Such approximation can lead to conservative behaviors, restricting their ability to navigate efficiently in complex environments. 

To improve upon the circle approximation, some researchers approximate robots or obstacles as ellipses (or ellipsoids in the case of a 3D space), which better represent their elongated shapes, although computing their distances is challenging. Early work in [19] proposed a conservative distance estimate between ellipsoids, formulated as an eigenvalue problem. Study [20] derived a closed-form expression 

that represents a distance metric between two ellipsoids in a 3D space. In [21], an approach for trajectory replanning of unmanned marine vehicles was adopted in which obstacles are modeled as ellipses that are augmented to consider the vehicle width. In [22], robots and obstacles were represented by sets of ellipsoids, and a point-world transformation is proposed to convert these ellipsoids to points to simplify collision avoidance. Some works use a mixture of circles and ellipses for shape approximation by either approximating the ego robot with a circle and its surrounding robots with ellipses [23], [24] or vice versa [25]. These approaches reduce conservatism compared to a purely circle-based approximation but still do not fully capture the actual shape of car-like robots. Note that among these works, CBFs are only used in [20] and [24]. 

# B. Paper Contributions

Our main contributions are threefold. 

1) We introduce a Minimum Translation Vector (MTV)- based, non-differentiable safety margin for collision avoidance of car-like robots that accounts for their actual geometries and headings. This safety margin enables a more precise estimation of safe regions and therefore reduces conservatism. 

2) We propose the notion of relative dynamics to enable tractable learning of this non-differentiable safety margin via a differentiable neural network, making it suitable as a candidate CBF. 

3) We conduct a case study in which we establish the theoretical foundation for applying our proposed safety margin to car-like robots modeled by a nonlinear kinematic bicycle model. 

Although the concept of MTV is widely used in collision avoidance [26], our work is the first to incorporate it into CBFs for collision avoidance of car-like robots. 

# C. Notation

A variable $x$ is annotated with a superscript $x ^ { i }$ if it belongs to robot i. A relative variable includes two letters in its superscript to indicate the direction, e.g., $x ^ { j i }$ denotes $x$ of robot $j$ relative to that of robot $i$ . If the relative variable is expressed in robot $i$ ’s ego perspective rather than in the global coordinate system, an underline is used, e.g., $x ^ { j \underline { { i } } }$ . Vectors are generally denoted in bold. The dot product of two vectors $\textbf { \em a }$ and $^ { b }$ is represented by ${ \mathbf { } } _ { a } . { \mathbf { } } _ { b }$ . Time arguments of time-varying variables are often omitted for brevity. 

# D. Paper Structure

Section II revisits the kinematic bicycle model and CBF. Section III introduces our MTV-based safety margin and its integration into CBF. Section IV presents our case study and establishes a theoretical foundation for applying the proposed safety margin for collision avoidance of car-like robots modeled by the kinematic bicycle model. Finally, Section V concludes and outlines future research directions. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/e42ec0cc9f92337ed5d419e84c328bb1fb820a78d2cb39f0d143b65a525779a5.jpg)



Fig. 2: The kinematic bicycle model. $C$ : center of gravity; $x , y \colon x \cdot$ - and $y$ -coordinates; $v$ : velocity; $\beta$ : slip angle; $\psi$ : yaw angle; $\delta$ : steering angle; $\ell _ { w b }$ : wheelbase; $\ell _ { r }$ : rear wheelbase.


# II. PRELIMINARIES

We consider an input-affine control system given by 

$$
\dot {\boldsymbol {x}} = f (\boldsymbol {x}) + g (\boldsymbol {x}) \boldsymbol {u}. \tag {1}
$$

Here, $f : \mathbb { R } ^ { n } \to \mathbb { R } ^ { n }$ and $g : \mathbb { R } ^ { n }  \mathbb { R } ^ { n \times m }$ are Lipschitz continuous. $\pmb { x } \in \mathcal { X } \subset \mathbb { R } ^ { n }$ is the state vector, and $\pmb { u } \in \mathcal { U } \subset$ $\mathbb { R } ^ { m }$ is the control input vector, where $n$ and $m$ denote the dimensions of the state space and control input space, and $\mathcal { X }$ and $\mathcal { U }$ are their respective admissible sets. 

# A. Kinematic Bicycle Model

We employ the kinematic bicycle model to model the dynamics of car-like robots in our case study, as it captures the essential dynamics required for motion planning and control [27], [28]. This model approximates the robot as a single-track model with two wheels, as depicted in Fig. 2. 

We define the state vector as $\pmb { x } : = [ x , y , \psi , v , \delta ] ^ { \top } \in \mathbb { R } ^ { 5 }$ , where $x$ and $y$ denote the position in the global coordinate system, and $\psi , v$ , and $\delta$ denote the heading (or yaw angle), speed, and steering angle, respectively. The control input vector is defined as $\pmb { u } : = [ u _ { v } , u _ { \delta } ] ^ { \top } \in \mathbb { R } ^ { 2 }$ , where $u _ { v }$ denotes the acceleration and $u _ { \delta }$ the steering rate. The dynamics of the kinematic bicycle model are given by 

$$
\dot {\boldsymbol {x}} = \left[ \begin{array}{c} v \cos (\psi + \beta) \\ v \sin (\psi + \beta) \\ \frac {v}{\ell_ {w b}} \tan (\delta) \cos (\beta) \\ 0 \\ 0 \end{array} \right] + \left[ \begin{array}{l l} 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ 1 & 0 \\ 0 & 1 \end{array} \right] \left[ \begin{array}{l} u _ {v} \\ u _ {\delta} \end{array} \right], \tag {2}
$$

where $\ell _ { w b } \in \mathbb { R }$ denotes the wheelbase of the vehicle and the slip angle $\beta \in \mathbb { R }$ is computed as 

$$
\beta = \tan^ {- 1} \left(\frac {\ell_ {r}}{\ell_ {w b}} \tan \delta\right), \tag {3}
$$

with $\ell _ { r } \in \mathbb { R }$ representing the rear wheelbase. 

# B. Control Barrier Functions

CBFs provide a tool to enforce safety constraints by ensuring that the system’s state remains within a safe set. Assume the safe set is defined as 

$$
C = \left\{\boldsymbol {x} \in \mathcal {X} \mid h (\boldsymbol {x}) \geq 0 \right\}, \tag {4}
$$

where $h : \mathcal { X }  \mathbb { R }$ is a continuously differentiable function. 

Definition 1 (Class $\kappa$ Function). A Lipschitz continuous function $\alpha : \mathbb { R }  \mathbb { R }$ is said to belong to class $\kappa$ if it is strictly increasing and satisfies $\alpha ( 0 ) = 0$ . 

Definition 2 (Control Barrier Function [2], [3]). Given a set $C$ as in (4), a continuously differentiable function $h : \mathcal { X }  \mathbb { R }$ is a candidate CBF for system (1) if there exists a class $\kappa$ function $\alpha$ such that 

$$
\sup  _ {\boldsymbol {u} \in \mathcal {U}} \dot {h} (\boldsymbol {x}, \boldsymbol {u}) \geq 0, \quad \forall \boldsymbol {x} \in C.
$$

Here, $\dot { h } ( { \pmb x } , { \pmb u } )$ denotes the time derivative of $h ( { \pmb x } )$ and is given by is equiva $\begin{array} { r } { \frac { d h ( \underline { { \dot { \boldsymbol { x } } } } ) } { d t } = \frac { \partial h ( \underline { { \boldsymbol { x } } } ) } { \partial \underline { { \boldsymbol { x } } } } \dot { \boldsymbol { x } } = \frac { \partial h ( \underline { { \boldsymbol { x } } } ) } { \partial \underline { { \boldsymbol { x } } } } \left( f ( \pmb { x } ) + g ( \pmb { x } ) \pmb { u } \right) } \end{array}$ $\begin{array} { r l } { \frac { \dot { d } h ( \pmb { x } ) } { \dot { d t } } } & { { } = } \end{array}$ $L _ { f } h ( { \pmb x } ) + L _ { g } h ( { \pmb x } ) { \pmb u }$ , with $L _ { f }$ and $L _ { g }$ denoting the Lie derivatives along $f$ and $g$ , respectively. 

Definition 3 (Forward Invariant). A set $C \subset { \mathcal { X } }$ is forward invariant for system (1) if for any initial state $\pmb { x } ( t _ { 0 } ) \in C$ , the solution satisfies $\pmb { x } ( t ) \in C$ for all $t \geq t _ { 0 }$ . 

In practice, CBFs often have a high relative degree. The relative degree of a continuously differentiable function w.r.t. a system is the number of times the function must be differentiated along the system dynamics until the control input explicitly appears. 

Definition 4 (Relative Degree [6]). A continuously differentiable function $h : \mathcal { X }  \mathbb { R }$ is said to have relative degree $r \in \mathbb N$ w.r.t. system (1) if for all $\textbf { \em x } \in { \mathcal { X } }$ , $L _ { g } L _ { f } ^ { i } h ( { \pmb x } ) =$ $0 , \forall i \in \{ 0 , 1 , \ldots , r - 2 \}$ , and $L _ { g } L _ { f } ^ { r - 1 } h ( { \pmb x } ) \neq 0$ . 

Let $h ( { \pmb x } )$ be continuously differentiable. Define 

$$
\Psi_ {0} (\boldsymbol {x}) := h (\boldsymbol {x}), \tag {5}
$$

and recursively define 

$$
\Psi_ {i} (\boldsymbol {x}) := \dot {\Psi} _ {i - 1} (\boldsymbol {x}) + \alpha_ {i} \left(\Psi_ {i - 1} (\boldsymbol {x})\right), \quad i \in \{1, \dots , r \}, \tag {6}
$$

where $\alpha _ { i } ( \cdot )$ is an $( r \mathrm { ~ - ~ } i ) \mathrm { t l }$ h times differentiable class $\kappa$ function. Furthermore, define 

$$
C _ {i} := \left\{\boldsymbol {x} \in \mathcal {X}: \Psi_ {i - 1} (\boldsymbol {x}) \geq 0 \right\}, \quad i \in \{1, \dots , r \}. \tag {7}
$$

Definition 5 (High-Order CBFs [6]). Given the sets $C _ { i }$ as in (7) and functions $\Psi _ { i }$ as in (6) for all $i \in \{ 1 , \ldots , r \}$ , a function $h : \mathbb { R } ^ { n }  \mathbb { R }$ is a candidate High-Order CBF (HOCBF) with relative degree $r$ for system (1) if it is at least rth times differentiable and there exist $( r - i )$ th times differentiable class $\kappa$ functions $\alpha _ { i }$ , for $i \in \{ 1 , \ldots , r \}$ , such that 

$$
\sup  _ {\boldsymbol {u} \in \mathcal {U}} \Psi_ {r} (\boldsymbol {x}) \geq 0, \quad \forall \boldsymbol {x} \in \bigcap_ {i = 1} ^ {r} C _ {i}. \tag {8}
$$

Theorem 1 (Thm. 4 in [6]). Given a candidate HOCBF $h ( { \pmb x } )$ for system (1) as in Definition 5, if the initial state satisfies $\textstyle { \pmb { x } } ( t _ { 0 } ) \in \bigcap _ { i = 1 } ^ { r } C _ { i }$ , then any Lipschitz continuous controller that satisfies (8) renders the set $\textstyle \bigcap _ { i = 1 } ^ { r } C _ { i }$ forward invariant for all $t \geq t _ { 0 }$ . 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/7831ee3ea906c4f7fbf33ea985b5131221c6a4082e8b5201920173a0b0ad4b6c.jpg)



Fig. 3: An example illustrating Algorithm 1 for the case where $g _ { x _ { A } ^ { i } } < 0$ , $g _ { y _ { A } ^ { i } } < 0$ , $g _ { x _ { A } ^ { j } } > 0$ , and $g _ { y _ { A } ^ { j } } < 0$ .


# III. MTV-BASED SAFETY MARGIN AS CBFS

In this section, we introduce an MTV-based safety margin for collision avoidance of car-like robots that takes into account their actual geometries and headings. We also describe how to incorporate this safety margin into a CBF framework. 

# A. MTV-Based Safety Margin

The Separating Axis Theorem is a fundamental concept in computational geometry for detecting collisions between convex shapes [29]. It states that two convex shapes do not intersect if there exists an axis along which their projections do not overlap. If no such axis exists, the shapes are colliding. In this context, the MTV represents the smallest vector required to separate the shapes [26]. We extend the concept of the MTV to define a safety margin for two car-like robots, which are approximated as rectangles. This safety margin represents the minimal movement required for one robot to make contact with the other, taking into account their geometries and headings. 

Algorithm 1 details the computation of the MTV-based safety margin for a given pair of rectangles $i$ and $j$ . For each rectangle $k \in \{ i , j \}$ and for each of its orthogonal axes $a ~ \in ~ \{ x _ { A } ^ { k } , y _ { A } ^ { k } \}$ , the algorithm projects the vertices of both rectangles onto axis $a$ . If the projections do not overlap on axis $a$ , the gap $g _ { a }$ is positive and represents the separation; otherwise, $g _ { a }$ is negative and corresponds he overlapping length. Based on the signs of , the algorithm classifies the relative position as $g _ { x _ { A } ^ { k } }$ andually $g _ { y _ { A } ^ { k } }$ separating, overlapping along one axis, or non-separating, and then computes a distance metric $d ^ { k }$ accordingly (lines 12 to 18). Finally, the safety margin $d _ { \mathrm { M T V } }$ is determined based on $d ^ { i }$ and $d ^ { j }$ (lines 20 to 26). Figure 3 illustrates a case where $g _ { x _ { A } ^ { i } } < 0$ , $g _ { y _ { A } ^ { i } } ~ < ~ 0$ , $g _ { x _ { \ A } ^ { j } } > 0$ , and $g _ { y _ { A } ^ { j } } < 0$ , resulting in $d ^ { i } = - \operatorname* { m i n } ( | g _ { x _ { A } ^ { i } } | , | g _ { y _ { A } ^ { i } } | ) ^ { * } < 0 .$ , $d ^ { j } = g _ { y _ { A } ^ { j } } ^ { - } > 0$ , and hence $d _ { \mathrm { M T V } } = \operatorname* { m a x } ( d ^ { i } , \ddot { d } ^ { j } ) = \ddot { d } ^ { j }$ . 

# B. Learning-Based Safety-Margin Approximator

To enable our MTV-based safety margin as a valid candidate CBF, we approximate it using a neural network to obtain a differentiable safety-margin approximator. 

Algorithm 1 requires the positions, headings, widths, and lengths of the two rectangles, which results in a tendimensional input space. This high dimensionality can make 


Algorithm 1 MTV-Based Safety Margin


Input: Positions, headings, widths, and lengths of rectangles $i$ and $j$ Output: Safety margin $d_{\mathrm{MTV}}$ 1: Compute the positions of both rectangles' vertices.   
2: for each rectangle $k\in \{i,j\}$ do   
3: Compute the pair of orthogonal axes $\{x_A^k,y_A^k\}$ of $k$ 4: for each axis $a\in \{x_A^k,y_A^k\}$ do   
5: Project both rectangles' vertices onto axis $a$ 6: if the projections do not overlap on axis $a$ then   
7: $g_{a}\gets$ gap between the projections (positive).   
8: else   
9: $g_{a}\gets$ overlapping length (negative).   
10: end if   
11: end for   
12: if both $g_{x_A^k},g_{y_A^k} > 0$ then Mutually separating   
13: $d^{k} = \sqrt{(g_{x_{A}^{k}})^{2} + (g_{y_{A}^{k}})^{2}}.$ 14: else if both $g_{x_A^k},g_{y_A^k} <   0$ then Separating by one rectangle's axes   
15: $d^{k} = -\min (|g_{x_{A}^{k}}|,|g_{y_{A}^{k}}|).$ 16: else Non-separating   
17: $d^{k} = \max (g_{x_{A}^{k}},g_{y_{A}^{k}}).$ 18: end if   
19: end for   
20: if both $d^i,d^j >0$ then $\triangleright i$ and $j$ are separated by both their axes   
21: $d_{\mathrm{MTV}} = \min (d^i,d^j)$ 22: else if both $d^i,d^j <  0$ then $\triangleright i$ and $j$ are unseparated   
23: $d_{\mathrm{MTV}} = -\min (|d^i|,|d^j|).$ 24: else One rectangle overlaps the other along its axes   
25: $d_{\mathrm{MTV}} = \max (d^i,d^j).$ 26: end if   
27: return dMTV 

learning intractable. To reduce the input space, we introduce the concept of relative dynamics by using the relative position and relative heading of one rectangle from the perspective of the other. We denote the relative dynamics as 

$$
\dot {\boldsymbol {x}} ^ {j \underline {{i}}} = f ^ {j \underline {{i}}} (\boldsymbol {x} ^ {j \underline {{i}}}) + g ^ {j \underline {{i}}} (\boldsymbol {x} ^ {j \underline {{i}}}) \boldsymbol {u}, \tag {9}
$$

where $f ^ { j \underline { { i } } } ~ : ~ \mathbb { R } ^ { 3 } ~ \to ~ \mathbb { R } ^ { 3 }$ and $g ^ { j \underline { { i } } } \colon \mathbb { R } ^ { 3 } \to \mathbb { R } ^ { 3 \times 4 }$ are Lipschitz continuous. The relative state is defined as $x ^ { j \underline { { { i } } } } : = $ $[ x ^ { j \underline { { i } } } , y ^ { j \underline { { i } } } , \psi ^ { j \underline { { i } } } ] ^ { \top } \ \in \ \mathcal { X } \ \subset \ \mathbb { R } ^ { 3 }$ , where $( x ^ { j \underline { { i } } } , y ^ { j \underline { { i } } } )$ denote the relative position and $\psi ^ { j \underline { { i } } }$ the relative heading of robot $j$ w.r.t. robot $i$ , as observed from $i$ ’s perspective. The joint control input is $\pmb { \mathscr { u } } : = \left[ ( \pmb { \mathscr { u } } ^ { i } ) ^ { \top } , ( \pmb { \mathscr { u } } ^ { j } ) ^ { \top } \right] ^ { \top } \in \mathcal { U } \subset \mathbb { R } ^ { 4 }$ , with $\boldsymbol { u } ^ { i } \in \mathbb { R } ^ { 2 }$ and $\boldsymbol { u } ^ { j } \in \mathbb { R } ^ { 2 }$ being the control inputs for robots $i$ and $j$ , respectively. We train a neural network $h _ { \theta } ( \pmb { x } ^ { j \underline { { i } } } ) : \mathbb { R } ^ { 3 }  \mathbb { R }$ , with parameters $\theta$ , to approximate the function computed by Algorithm 1. Note that the width and length of the rectangles are not included in the network input because they are timeinvariant, and the network can implicitly learn the geometric relationships. 

# C. Construction of the Control Barrier Function

We employ the MTV-based safety-margin approximator $h _ { \theta } ( \pmb { x } ^ { j \underline { { i } } } )$ as a candidate CBF. To account for the approximation error between $h _ { \theta }$ and the true value computed by Algorithm 1, we consider an upper bound on the approximation error, $e _ { \mathrm { m a x } } > 0$ . We construct the CBF as 

$$
h _ {\mathrm {M T V}} \left(\boldsymbol {x} ^ {j i}\right) = h _ {\theta} \left(\boldsymbol {x} ^ {j i}\right) - e _ {\max } \tag {10}
$$

Although considering the upper bound of the error can introduce conservatism, this is acceptable if it is sufficiently small. 

Assumption 1. The upper bound of the approximation error $e _ { \mathrm { m a x } }$ in (10) is known. 

We note that Assumption 1 is reasonable because the input space of the neural network can be restricted to a known range. This limitation prevents out-of-distribution issues and allows a thorough evaluation of all possible inputs. The three-dimensional input space comprises the relative position $( x ^ { j \underline { { i } } } , y ^ { j \underline { { i } } } )$ and the relative heading $\psi ^ { j \underline { { i } } }$ , where the heading is naturally bounded within $[ - \pi , \pi ]$ . We limit the relative position to a small yet practical range around robot $i$ . If robot $j$ is outside this range, we revert to the C2C-based safety margin, for which the conservatism has a negligible impact. 

Assumption 2. The function $h _ { \mathrm { M T V } } ( \pmb { x } ^ { j \underline { { i } } } )$ in (10) is at least rth times differentiable, where $r$ is its relative degree w.r.t. system (1). 

This assumption is justified since, according to Definition 5, a candidate HOCBF with relative degree $r$ must be at least rth times differentiable. Continuous differentiability can be ensured by using a continuously differentiable activation function, such as the TanH function, in the neural network. 

Theorem 2. Let $r$ denote the relative degree of $h _ { \mathrm { M T V } }$ in (10) w.r.t. system (1). Redefine $\Psi _ { 0 }$ as in (5), $\Psi _ { i }$ as in (6), and $C _ { i }$ as in (7) using $h _ { \mathrm { M T V } }$ , for all $i \in \{ 1 , \ldots , r \}$ . If the initial state satisfies $\pmb { x } ^ { j \underline { { i } } } ( t _ { 0 } ) \in \bigcap _ { i = 1 } ^ { r } C _ { i }$ , then any Lipschitz continuous controller that satisfies 

$$
\sup  _ {\boldsymbol {u} \in \mathcal {U}} \Psi_ {r} \left(\boldsymbol {x} ^ {j i} (t)\right) \geq 0, \quad \forall \boldsymbol {x} ^ {j i} (t) \in \bigcap_ {i = 1} ^ {r} C _ {i}, \tag {11}
$$

renders system (9) collision-free for all $t \geq t _ { 0 }$ 

Proof. Since $h _ { \mathrm { M T V } }$ is at least rth times differentiable, it qualifies as a candidate HOCBF with relative degree $r$ . By [6, Thm. 4], the set $C _ { 1 }$ is forward invariant for system (9). This implies that the state $\pmb { x } ^ { j \underline { { i } } } ( t )$ remains in $C _ { 1 }$ , i.e., $h _ { \mathrm { M T V } } ( \pmb { x } ^ { j \underline { { { i } } } } ) ~ \geq ~ 0$ for all $t \geq t _ { 0 }$ . With the “conservative” consideration of the upper bound $e _ { \mathrm { m a x } }$ of the approximation error in (10), the safety margin between robots $i$ and $j$ remains non-negative, ensuring that system (9) remains collision-free for all $t \geq t _ { 0 }$ . □ 

Remark 1. Although the system defined in (9) involves only two robots, Theorem 2 can be extended to systems with any number of robots. For each pair of robots, we can construct 

a sub-system as in (9), resulting in ${ \binom { k } { 2 } } = k ( k - 1 ) / 2$ subsystems for $k$ robots. The overall system remains collisionfree as long as all sub-systems are collision-free. 

# IV. CASE STUDY

In this section, we conduct a case study to numerically evaluate the proposed MTV-based safety margin in simulations with two car-like robots. Without loss of generality, we employ the kinematic bicycle model [27] to model the robots’ dynamics. We design two scenarios—an overtaking scenario and a bypassing scenario—to compare our MTVbased safety margin with the traditional C2C-based safety margin. Codes for reproducing our experimental results and video demonstrations are available in our open-source repository1. 

First, we establish the theoretical foundation for applying our proposed notion of relative dynamics to the kinematic bicycle model in Section IV-A. Then, we describe how to train the MTV-based safety margin approximator and formulate the Optimal Control Problem (OCP) in Section IV-B. Section IV-C presents the experimental results for the overtaking scenario, and Section IV-D those for the bypassing scenario. 

# A. Relative Dynamics of the Kinematic Bicycle Model

We derive the explicit form of the relative dynamics (9) for two car-like robots $i$ and $j$ modeled by the kinematic bicycle model (2). Without loss of generality, we designate robot $i$ as the ego robot and express the state of robot $j$ relative to $i$ . The relative state in the global coordinate system is 

$$
\boldsymbol {x} ^ {j i} = \boldsymbol {x} ^ {j} - \boldsymbol {x} ^ {i}, \tag {12}
$$

where $\mathbf { } \mathbf { } x ^ { i } : = \left[ x ^ { i } , y ^ { i } , \psi ^ { i } \right] ^ { \intercal }$ and $\mathbf { } \mathbf { } x ^ { j } : = \left[ x ^ { j } , y ^ { j } , \psi ^ { j } \right] ^ { \intercal }$ . We project $\pmb { x } ^ { j i }$ into robot $i$ ’s ego coordinate system, yielding 

$$
\boldsymbol {x} ^ {j \underline {{i}}} = \left[ \begin{array}{c} x ^ {j \underline {{i}}} \\ y ^ {j \underline {{i}}} \\ \psi^ {j \underline {{i}}} \end{array} \right] = \left[ \begin{array}{c} x ^ {j i} \cos \psi^ {i} + y ^ {j i} \sin \psi^ {i} \\ - x ^ {j i} \sin \psi^ {i} + y ^ {j i} \cos \psi^ {i} \\ \psi^ {j i} \end{array} \right]. \tag {13}
$$

By differentiating (13) w.r.t. time, we obtain the first time derivative $\dot { \pmb { x } } ^ { j \underline { { i } } } : = [ \dot { \dot { x } } ^ { j \underline { { i } } } , \dot { y } ^ { j \underline { { i } } } , \dot { \psi } ^ { j \underline { { i } } } ] ^ { \top }$ as 

$$
\dot {x} ^ {j i} = \cos \psi^ {i} \dot {x} ^ {j i} - \sin \psi^ {i} x ^ {j i} \dot {\psi} ^ {i} + \sin \psi^ {i} \dot {y} ^ {j i} + \cos \psi^ {i} y ^ {j i} \dot {\psi} ^ {i},
$$

$$
\dot {y} ^ {j \dot {i}} = \cos \psi^ {i} \dot {y} ^ {j \dot {i}} - \sin \psi^ {i} y ^ {j i} \dot {\psi} ^ {i} - \sin \psi^ {i} \dot {x} ^ {j i} - \cos \psi^ {i} x ^ {j i} \dot {\psi} ^ {i}, \tag {14}
$$

$$
\dot {\psi} ^ {j \dot {i}} = \dot {\psi} ^ {j \dot {i}}.
$$

Similarly, applying the product rule to (14) yields the second time derivative $\ddot { \pmb { x } } ^ { j i } : = [ \ddot { x } ^ { j \underline { { { i } } } } , \ddot { y } ^ { j \underline { { { i } } } } , \ddot { \psi } ^ { j \underline { { { i } } } } ] ^ { \top }$ as 

$$
\begin{array}{l} \ddot {x} ^ {j \dot {\bar {i}}} = \cos \psi^ {i} \dot {x} ^ {j i} - 2 \sin \psi^ {i} \dot {x} ^ {j i} \dot {\psi} ^ {j} - x ^ {j i} \cos \psi^ {i} (\dot {\psi} ^ {i}) ^ {2} \\ - x ^ {j i} \sin \psi^ {i} \ddot {\psi} ^ {i} + \sin \psi^ {i} \ddot {y} ^ {j i} + 2 \cos \psi^ {i} \dot {y} ^ {j i} \dot {\psi} ^ {i} \\ - y ^ {j i} \sin \psi^ {i} (\dot {\psi} ^ {i}) ^ {2} + y ^ {j i} \cos \psi^ {i} \ddot {\psi} ^ {i}, \\ \end{array}
$$

$$
\begin{array}{l} \dot {y} ^ {j \dot {i}} = \cos \psi^ {i} \dot {y} ^ {j i} - 2 \sin \psi^ {i} \dot {y} ^ {j i} \dot {\psi} ^ {i} - y ^ {j i} \cos \psi^ {i} (\dot {\psi} ^ {i}) ^ {2} \\ - y ^ {j i} \sin \psi^ {i} \ddot {\psi} ^ {i} - \sin \psi^ {i} \dot {x} ^ {j i} - 2 \cos \psi^ {i} \dot {x} ^ {j i} \dot {\psi} ^ {i} \\ + x ^ {j i} \sin \psi^ {i} (\dot {\psi} ^ {i}) ^ {2} - x ^ {j i} \cos \psi^ {i} \ddot {\psi} ^ {i}, \\ \end{array}
$$

$$
\ddot {\psi} ^ {j i} = \ddot {\psi} ^ {j i}.
$$

1github.com/bassamlab/sigmarl 

We compute the time derivatives of $\pmb { x } ^ { j i }$ as 

$$
\dot {\boldsymbol {x}} ^ {j i} := \dot {\boldsymbol {x}} ^ {j} - \dot {\boldsymbol {x}} ^ {i} \quad \text {a n d} \quad \ddot {\boldsymbol {x}} ^ {j i} := \ddot {\boldsymbol {x}} ^ {j} - \ddot {\boldsymbol {x}} ^ {i}. \tag {15}
$$

From (2), we obtain $\dot { \pmb x } ^ { i } : = [ \dot { x } ^ { i } , \dot { y } ^ { i } , \dot { \psi } ^ { i } ] ^ { \top } \ \in \ \mathbb { R } ^ { 3 }$ . Differentiating this yields $\ddot { \pmb { x } } ^ { i } : = [ \ddot { x } ^ { i } , \ddot { y } ^ { i } , \ddot { \psi } ^ { i } ] ^ { \top }$ as (omitting the superscript $i$ for brevity) 

$$
\left[ \begin{array}{c} \ddot {x} ^ {i} \\ \ddot {y} ^ {i} \\ \ddot {\psi} ^ {i} \end{array} \right] = \left[ \begin{array}{c} u _ {v} \cos (\psi + \beta) - v \sin (\psi + \beta) (\dot {\psi} + \dot {\beta}) \\ u _ {v} \sin (\psi + \beta) + v \cos (\psi + \beta) (\dot {\psi} + \dot {\beta}) \\ \frac {\cos \beta}{\ell_ {w b}} \left(u _ {v} \tan \delta + v \sec^ {2} \delta u _ {\delta} - v \tan \beta \tan \delta \dot {\beta}\right) \end{array} \right], \tag {16}
$$

where $\beta$ is computed as in (3), $\begin{array} { r } { \dot { \beta } ~ = ~ \frac { k \sec ^ { 2 } \delta } { 1 + ( k \tan \delta ) ^ { 2 } } u _ { \delta } } \end{array}$ , with $k : = \ell _ { r } / \ell _ { w b }$ and $\sec \delta : = 1 / \cos \delta$ . A similar computation applies to robot $j$ . 

Theorem 3. Function $h _ { \mathrm { M T V } } ( \pmb { x } ^ { j \underline { { i } } } )$ defined in (10) w.r.t. system (9) has relative degree two. 

Proof. We determine the relative degree by counting the number of times $h _ { \mathrm { M T V } } ( \pmb { x } ^ { j \underline { { i } } } )$ must be differentiated along the system dynamics until the control input $\textbf { \em u }$ appears. The first time derivative is given by 

$$
\dot {h} _ {\mathrm {M T V}} \left(\boldsymbol {x} ^ {j \underline {{i}}}\right) = \dot {h} _ {\theta} \left(\boldsymbol {x} ^ {j \underline {{i}}}\right) = \nabla h _ {\theta} ^ {\top} \dot {\boldsymbol {x}} ^ {j \underline {{i}}}, \tag {17}
$$

where $\begin{array} { r } { \nabla h _ { \theta } : = \left[ \frac { \partial h _ { \theta } } { \partial x ^ { j \underline { { i } } } } , \frac { \partial h _ { \theta } } { \partial y ^ { j \underline { { i } } } } , \frac { \partial h _ { \theta } } { \partial \psi ^ { j \underline { { i } } } } \right] ^ { \top } } \end{array}$ is the gradient vector of $h _ { \theta }$ . Since neither $\nabla h _ { \theta }$ nor $\dot { \pmb { x } } ^ { j \dot { \underline { { i } } } }$ includes the control input, the first derivative does not depend on $\textbf { \em u }$ . Differentiating once more and applying the product rule gives 

$$
\ddot {h} _ {\mathrm {M T V}} \left(\boldsymbol {x} ^ {j _ {2}}\right) = \nabla h _ {\theta} ^ {\top} \ddot {\boldsymbol {x}} ^ {j _ {i}} + \dot {\boldsymbol {x}} ^ {j _ {i} \top} H _ {h _ {\theta}} \dot {\boldsymbol {x}} ^ {j _ {2}}, \tag {18}
$$

where $H _ { h _ { \theta } } ~ \in ~ \mathbb { R } ^ { 3 \times 3 }$ is the Hessian matrix of $h _ { \theta }$ . Since $\ddot { \pmb { x } } ^ { j i }$ is computed from ${ \ddot { x } } ^ { i }$ and ${ \ddot { \mathbf { x } } ^ { j } }$ , which explicitly involve the control inputs $u _ { v }$ and $u _ { \delta }$ (see (16)), the control input appears in the second derivative. Thus, $h _ { \mathrm { M T V } } ( \pmb { x } ^ { j \underline { { i } } } )$ has relative degree two. □ 

Note that for a given neural network and input vector, the gradient and Hessian can be computed directly. We omit the details due to space limitations. 

Remark 2. Although we use the kinematic bicycle model in this derivation, the proposed notion of relative dynamics can be applied to any dynamical model. The derivation procedure is similar to our derivation in this section. 

# B. Optimal Control Problem Formulation

1) Training the Safety-Margin Approximator: We generate a training dataset by computing the MTV-based safety margin using Algorithm 1. We create a dense grid of inputs over the feature space as 

$$
\left[ x ^ {j \underline {{i}}}, y ^ {j \underline {{i}}}, \psi^ {j \underline {{i}}} \right] \in \left[ - 3 \ell_ {w b}, 3 \ell_ {w b} \right] \times \left[ - 3 \ell_ {w b}, 3 \ell_ {w b} \right] \times \left[ - \pi , \pi \right].
$$

We limit the relative position of robot $j$ to this space to allow for an estimable upper bound on the approximation error (see Assumption 1). We fix robot $i$ ’s position and heading at zero and densely enumerate robot $j$ ’s position and heading over the feature space. Then, we run Algorithm 1 with the positions and headings of the robots to obtain the MTV-based distances, which serve as labels. Our training 

dataset contains approximately $8 0 \mathrm { k }$ uniformly distributed data points. We train a simple fully connected neural network $h _ { \theta }$ with two hidden layers of 62 nodes each and TanH activation functions. We test on a separate dataset of 20k points, yielding a maximum approximation error of $0 . 0 1 2 1 \mathrm { m }$ (corresponding to $1 5 . 2 \%$ of the robot’s width) and a mean error of $2 . 7 8 \%$ of the robot’s width. Increasing the size of the testing dataset produces similar results. 

2) OCP: We use the learned safety-margin approximator $h _ { \theta }$ as the candidate CBF and incorporate it into a Quadratic Program (QP) to modify an unsafe nominal controller, resulting in a CBF-QP formulation. The goal is to adjust the nominal control actions minimally to ensure safety. We employ an RL policy trained using our SigmaRL [30], [31], an open-source multi-agent RL framework for motion planning of CAVs, as the nominal controller. Henceforth, we refer to the robots as vehicles. At each time step, the nominal controller receives a short-term waypoint-based reference path and outputs control actions to follow it. We purposely train the policy to be greedy in following the reference path without considering collisions. We formulate the OCP as a CBF-QP as 

$$
J (\boldsymbol {u}) = \min  _ {\boldsymbol {u}} \left(\boldsymbol {u} - \boldsymbol {u} _ {\text {n o m}}\right) ^ {\top} Q \left(\boldsymbol {u} - \boldsymbol {u} _ {\text {n o m}}\right), \tag {19a}
$$

$\begin{array} { r } { \mathrm { ~ 3 . t . ~ } \quad \Psi _ { 2 } \bigl ( \mathbf { { r } } ^ { j \underline { { i } } } \bigr ) \geq 0 \quad \bigl ( \sec \mathrm { ~ ( 8 ) } \bigr ) , } \end{array}$ (19b) 

$$
\boldsymbol {u} _ {\min } \leq \boldsymbol {u} \leq \boldsymbol {u} _ {\max }, \tag {19c}
$$

where ${ \bf { u } } _ { \mathrm { { n o m } } }$ denotes the nominal control action, and $Q \in$ $\mathbb { R } ^ { 4 \times 4 }$ is a weighting matrix. Constraint (19b) is the CBF condition for collision avoidance. We use $\Psi _ { 2 }$ since the candidate CBF has relative degree two, as shown in Theorem 3. For simplicity, we choose the same linear class $\kappa$ functions, with $\alpha _ { 1 } ( h ) = k _ { \alpha } h$ and $\alpha _ { 2 } ( h ) = k _ { \alpha } h$ , where $k _ { \alpha } > 0$ , yielding 

$$
\Psi_ {2} (\boldsymbol {x} ^ {j \dot {i}}) := \ddot {h} _ {\mathrm {M T V}} (\boldsymbol {x} ^ {j \dot {i}}) + 2 k _ {\alpha} \dot {h} _ {\mathrm {M T V}} (\boldsymbol {x} ^ {j \dot {i}}) + k _ {\alpha} ^ {2} h _ {\mathrm {M T V}} (\boldsymbol {x} ^ {j \dot {i}}).
$$

Constraint (19c) enforces the control inputs within their physical limits, with $\pmb { u } _ { \mathrm { m i n } }$ and $\pmb { u } _ { \mathrm { m a x } }$ representing the lower and upper bounds. We solve (19) iteratively in a discrete-time manner using the Python package CVXPY [32]. 

Table I lists the simulation parameters, and Figure 4 shows the flow diagram of our CBF-QP formulation using the learned MTV-based safety margin. 

We also compare our MTV-based safety margin with the traditional C2C-based safety margin. For the latter, we use 

$$
h _ {\mathrm {C} 2 \mathrm {C}} \left(\boldsymbol {x} ^ {j \underline {{i}}}\right) := \sqrt {\left(x ^ {j \underline {{i}}}\right) ^ {2} + \left(y ^ {j \underline {{i}}}\right) ^ {2}} - 2 r _ {\min }, \tag {20}
$$

where $r _ { \mathrm { m i n } } : = \sqrt { \ell ^ { 2 } + w ^ { 2 } } / 2$ is the minimum radius required to enclose the vehicle (with $\ell$ and $w$ being the vehicle’s length and width, respectively). One can easily verify that (20) is a valid HOCBF with relative degree two. 

# C. First Experiment: Overtaking

In this scenario, vehicle $i$ (blue) is the ego vehicle attempting to overtake a slower-moving vehicle $j$ (green) ahead, as depicted in Fig. 5 and Fig. 6. Vehicle $i$ uses 


TABLE I: Simulation parameters.


<table><tr><td>Parameter</td><td>Value</td></tr><tr><td>Length ℓ, width w</td><td>0.16 m, 0.08 m</td></tr><tr><td>Wheelbase ℓwb, rear wheelbase ℓr</td><td>0.16 m, 0.08 m</td></tr><tr><td>Max. (min.) acceleration uv,max</td><td>20 m/s2(−20 m/s2)</td></tr><tr><td>Max. (min.) steering rate uδ,max</td><td>16 rad/s (−16 rad/s)</td></tr><tr><td>Weighting matrix Q</td><td>I4×4 (identity matrix)</td></tr></table>

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/719fa0bb5c12af9473dc67f94894db1337071dbf1b6728bbe90d5f8b92d4128e.jpg)



Fig. 4: Flow diagram of the CBF-QP formulation with the MTV-based safety margin.


a trained RL-based nominal controller that directs it to move at approximately $1 . 0 \mathrm { m / s }$ and a CBF to ensure safety. Conversely, vehicle $j$ only uses a trained RL-based nominal controller, which maintains a constant speed of $0 . 5 \mathrm { m } / \mathrm { s }$ . To encourage overtaking, vehicle $i$ ’s reference path is projected to the centerline of the adjacent lane. In addition, to test the robustness of the CBF, vehicle $j$ conditionally switches lanes to obstruct the overtaking maneuver until it has obstructed three times, after which it remains in its lane. In this scenario, because vehicle $j$ follows its nominal actions, the OCP (19) has only two decision variables corresponding to vehicle i’s control actions. At each time step, vehicle $j$ communicates its nominal actions to vehicle $i$ , which uses them to formulate Constraint (19b) for collision avoidance. We ignore communication delays. 

Results: Figure 5 shows the performance with the C2Cbased safety margin. Vehicle $i$ starts at $x = - 1 . 2 \mathrm { m }$ and vehicle $j$ at $\textit { x } = \ - 0 . 4 \mathrm { m }$ . During the three obstructive maneuvers, the C2C-based margin prevents a collision, but after $t = 4 . 8 \mathrm { s }$ , vehicle $j$ stops obstructing, and vehicle i is unable to complete the overtaking maneuver because of the conservatism of the C2C-based margin. The blue line in Fig. 5 shows the $h$ value over time, with near-zero values indicating that the system state remains close to the boundary of the safe set. In contrast, as shown in Fig. 6, using our MTV-based safety margin, vehicle $i$ successfully overtakes vehicle $j$ at $t = 6 . 4 \mathrm { s }$ . 

# D. Second Experiment: Bypassing

In this scenario, two vehicles approach each other from opposite directions on a narrow road, as depicted in Fig. 7 and Fig. 8. Both vehicles use the same RL-based nominal controller that controls them to move at a speed of $1 . 0 \mathrm { m / s }$ . We apply the CBF to both vehicles to adjust the nominal control actions to ensure collision-freeness. Therefore, the centralized OCP (19) has four decision variables, two for each vehicle’s actions. 

Let $Y = y _ { 0 }$ denote a horizontal line at $y _ { 0 } \in \mathbb { R }$ . Initially, we project the nominal reference points for both vehicles 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/10a4bd368aed66524955f50540fb8ecd0c002a9cc993357d376f66fa4e5ea8e7.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/3f052d8ead220f00ecf08307b79b331e2074f1f0ab1de5f0bebc95d8e224b216.jpg)



Fig. 5: Overtaking scenario with C2C-based safety margin. The lowest safety margin occurs for $t \geq 5 . 4 \mathrm { s }$ .


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/ca88b90a2025108c7b3f2bad76dd1727d90ecacd675cc5a63b742eaf958734ec.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/94c2d4f3116cb91956d4907d52364da7b91d6313ed28008adca5bbe1f4d279a9.jpg)



Fig. 6: Overtaking scenario with MTV-based safety margin. The lowest safety margin occurs at $t = 6 . 4 \ : \mathrm { s }$ .


onto $Y ^ { i } ~ = ~ 0$ and $Y ^ { j } ~ = ~ 0$ , respectively. As the vehicles approach, the projections are shifted to $Y ^ { i } = y _ { \mathrm { n o m } } > 0$ and $Y ^ { j } = - y _ { \mathrm { n o m } } < 0$ to encourage bypassing. The parameters $y _ { \mathrm { n o m } }$ and $k _ { \alpha }$ are tuned jointly for optimal bypassing with minimal lateral displacement. Final values are $y _ { \mathrm { n o m } } = 0 . 1 1 6 \mathrm { m }$ $( 1 4 5 . 3 \%$ of the vehicle width) and $k _ { \alpha } = 3$ for the C2Cbased margin, and $y _ { \mathrm { n o m } } = 0 . 0 7 2 \mathrm { m }$ $( 9 0 . 0 \%$ of the vehicle width) and $k _ { \alpha } = 6$ for the MTV-based margin. 

Results: As shown in Fig. 7, vehicle $i$ starts at $x \quad =$ $- 1 . 2 \mathrm { m }$ moving rightward and vehicle $j$ at $\begin{array} { r l r } { x } & { { } = } & { 1 . 2 \mathrm { m } } \end{array}$ moving leftward. At $t = 2 . 5 \mathrm { s }$ , the safety margin reaches its minimum and the vehicles bypass each other, albeit with significant lateral evasion (vehicle i: $1 1 9 . 6 \%$ of the vehicle width, vehicle j: $1 2 4 . 7 \%$ ; average: $1 2 2 . 2 \%$ ). The bypassing process, i.e., both vehicles reach their opposite sides, takes about 3.6 s. In comparison, with our MTV-based safety margin, the minimum safety margin occurs at $t ~ = ~ 1 . 7 \mathrm { s }$ , and the vehicles bypass successfully with an average lateral evasion of $8 3 . 1 3 \%$ of the vehicle width (a reduction of $3 3 . 5 \%$ compared to the C2C-based safety margin), and the bypass completes at $t = 3 . 0 \mathrm { s }$ , which is $1 6 . 7 \%$ faster. 

# E. Discussions and Limitations

The overtaking and bypassing scenarios demonstrate that our MTV-based safety margin yields less conservative col-

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/c0adaa8665a3e61b8ac42222d76dac53516f0a27a3adfeed0c79c683b6d860bd.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/10f43ffda388afb780977610aa3e8d724e537f2ac48602c81595ad0324587e83.jpg)



Fig. 7: Bypassing scenario with C2C-based safety margin. The minimum safety margin occurs at $t = 2 . 5 \ : \mathrm { s }$ .


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/b951d242588164b3a791170b872995b7a2e159b2264611655a437144acd05072.jpg)


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-07/9b26e7cc-1e2a-43a9-afe2-54de1b3bc048/c5f3e29d9264da5bea867e6628e9555882a152c127f8cf5af9e064a4f60fcef1.jpg)



Fig. 8: Bypassing scenario with MTV-based safety margin. The minimum safety margin occurs at $t = 1 . 7 \mathrm { s }$ .


lision avoidance compared to the traditional C2C-based approach. In the overtaking scenario, the C2C-based safety margin prevents collisions but restricts overtaking, while our MTV-based safety margin allows a smooth, safe, and efficient overtaking maneuver. In the bypassing scenario, both methods maintain safety, yet our MTV-based margin reduces the lateral space required for bypassing by $3 3 . 5 \%$ and decreases bypassing time by $1 6 . 7 \%$ . Importantly, these improvements do not significantly affect the computation time. In the overtaking scenario, the OCP (19) is solved in an average of 7.4 ms per step with the C2C-based margin and 7.6 ms with the MTV-based margin, while in the bypassing scenario, the average times are 11.3 ms and 11.6 ms, respectively. 

Applying our HOCBF requires computing the gradient and Hessian matrix of the safety margin approximator (see (17) and (18)). Since the original function for the MTV-based margin is non-differentiable, we can not compute their actual values and do not know how accurate they are. Nevertheless, given the marginal approximation error in the safety margin $( 1 . 4 \%$ of the robot’s width on average) and its continuous nature, we expect the gradient and Hessian approximation errors to be similarly minor and negligible. 

# V. CONCLUSIONS

We proposed an MTV-based safety margin for collision avoidance of car-like robots that accounts for their actual geometries and headings, offering a more precise estimation of safe regions compared to the traditional C2C-based safety margin that approximates robots as circles. Because this safety margin is inherently non-differentiable, we approximated it with a neural network and introduced a notion of relative dynamics to facilitate tractable learning. We provided the theoretical foundation for applying this notion to a nonlinear kinematic bicycle model and compared our MTVbased safety margin with the traditional C2C-based safety margin through numerical experiments in overtaking and bypassing scenarios. In the overtaking scenario, while the traditional approach failed to complete the maneuver, our method succeeded. In the bypassing scenario, our approach reduced the required lateral space for bypassing by $3 3 . 5 \%$ and shortened the bypass time by $1 6 . 7 \%$ . Our approach achieved all these enhancements without significantly increasing computation time. Future work will extend our approach to multi-robot systems. 

# REFERENCES



[1] X. Zhang, A. Liniger, and F. Borrelli, “Optimization-based collision avoidance,” IEEE Transactions on Control Systems Technology, vol. 29, no. 3, pp. 972–983, 2021. 





[2] A. D. Ames, X. Xu, J. W. Grizzle, and P. Tabuada, “Control barrier function based quadratic programs for safety critical systems,” IEEE Transactions on Automatic Control, vol. 62, no. 8, pp. 3861–3876, 2017. 





[3] A. D. Ames, J. W. Grizzle, and P. Tabuada, “Control barrier function based quadratic programs with application to adaptive cruise control,” in 53rd IEEE Conference on Decision and Control, 2014, pp. 6271– 6278. 





[4] Y. Chen, H. Peng, and J. Grizzle, “Obstacle avoidance for low-speed autonomous vehicles with barrier function,” IEEE Transactions on Control Systems Technology, vol. 26, no. 1, pp. 194–206, 2017. 





[5] L. Wang, A. D. Ames, and M. Egerstedt, “Safety barrier certificates for collisions-free multirobot systems,” IEEE Transactions on Robotics, vol. 33, no. 3, pp. 661–674, 2017. 





[6] W. Xiao and C. Belta, “Control barrier functions for systems with high relative degree,” in 2019 IEEE 58th Conference on Decision and Control (CDC), 2019, pp. 474–479. 





[7] J. Xu and B. Alrifaee, “High-order control barrier functions: Insights and a truncated Taylor-based formulation,” arXiv preprint arXiv:2503.15014, 2025. 





[8] W. Xiao, T.-H. Wang, R. Hasani, M. Chahine, A. Amini, X. Li, and D. Rus, “BarrierNet: Differentiable control barrier functions for learning of safe robot control,” IEEE Transactions on Robotics, vol. 39, no. 3, pp. 2289–2307, 2023. 





[9] Z. Zhang, S. Han, J. Wang, and F. Miao, “Spatial-temporal-aware safe multi-agent reinforcement learning of connected autonomous vehicles in challenging scenarios,” in 2023 IEEE International Conference on Robotics and Automation (ICRA), 2023, pp. 5574–5580. 





[10] S. Han, S. Zhou, J. Wang, L. Pepin, C. Ding, J. Fu, and F. Miao, “A multi-agent reinforcement learning approach for safe and efficient behavior planning of connected autonomous vehicles,” IEEE Transactions on Intelligent Transportation Systems, vol. 25, no. 5, pp. 3654– 3670, 2024. 





[11] Y. Chen, A. Singletary, and A. D. Ames, “Guaranteed obstacle avoidance for multi-robot operations with limited actuation: A control barrier function approach,” IEEE Control Systems Letters, vol. 5, no. 1, pp. 127–132, 2020. 





[12] A. Singletary, K. Klingebiel, J. Bourne, A. Browning, P. Tokumaru, and A. Ames, “Comparative analysis of control barrier functions and artificial potential fields for obstacle avoidance,” in 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2021, pp. 8129–8136. 





[13] Z. Gao, G. Yang, and A. Prorok, “Online control barrier functions for decentralized multi-agent navigation,” in 2023 International Symposium on Multi-Robot and Multi-Agent Systems (MRS). IEEE, 2023, pp. 107–113. 





[14] S. Han, M. Vahs, and J. Tumova, “Risk-aware robot control in dynamic environments using belief control barrier functions,” arXiv preprint arXiv:2504.04097, 2025. 





[15] J. Ji, A. Khajepour, W. W. Melek, and Y. Huang, “Path planning and tracking for vehicle collision avoidance based on model predictive control with multiconstraints,” IEEE Transactions on Vehicular Technology, vol. 66, no. 2, pp. 952–964, 2016. 





[16] B. Alrifaee, “Networked model predictive control for vehicle collision avoidance,” Ph.D. dissertation, RWTH Aachen University, Aachen, Germany, 2017. 





[17] P. Scheffe, M. V. A. Pedrosa, K. Flaßkamp, and B. Alrifaee, “Receding horizon control using graph search for multi-agent trajectory planning,” IEEE Transactions on Control Systems Technology, vol. 31, no. 3, pp. 1092–1105, 2023. 





[18] P. Scheffe, T. M. Henneken, M. Kloock, and B. Alrifaee, “Sequential convex programming methods for real-time optimal trajectory planning in autonomous vehicle racing,” IEEE Transactions on Intelligent Vehicles, vol. 8, no. 1, pp. 661–672, 2023. 





[19] E. Rimon and S. P. Boyd, “Obstacle collision detection using best ellipsoid fit,” Journal of Intelligent and Robotic Systems, vol. 18, no. 2, pp. 105–126, 1997. 





[20] C. K. Verginis and D. V. Dimarogonas, “Closed-form barrier functions for multi-agent ellipsoidal systems with uncertain lagrangian dynamics,” IEEE Control Systems Letters, vol. 3, no. 3, pp. 727–732, 2019. 





[21] T. Glotzbach, B. Alrifaee, M. Schneider, M. Jacobi, A. Zimmermann, and C. Ament, “Advanced trajectory planning for obstacle avoidance of multiple unmanned marine vehicles (mumvs),” IFAC Proceedings Volumes, vol. 43, no. 20, pp. 354–359, 2010. 





[22] H. Tanner, S. Loizou, and K. Kyriakopoulos, “Nonholonomic navigation and control of cooperating mobile manipulators,” IEEE Transactions on Robotics and Automation, vol. 19, no. 1, pp. 53–64, 2003. 





[23] W. Schwarting, J. Alonso-Mora, L. Paull, S. Karaman, and D. Rus, “Safe nonlinear trajectory generation for parallel autonomy with a dynamic vehicle model,” IEEE Transactions on Intelligent Transportation Systems, vol. 19, no. 9, pp. 2994–3008, 2018. 





[24] Z. Jian, Z. Yan, X. Lei, Z. Lu, B. Lan, X. Wang, and B. Liang, “Dynamic control barrier function-based model predictive control to safety-critical obstacle-avoidance of mobile robot,” in 2023 IEEE International Conference on Robotics and Automation (ICRA), 2023, pp. 3679–3685. 





[25] H. Liu, Z. Huang, Z. Zhu, Y. Li, S. Shen, and J. Ma, “Improved consensus admm for cooperative motion planning of large-scale connected autonomous vehicles with limited communication,” IEEE Transactions on Intelligent Vehicles, pp. 1–17, 2024. 





[26] C. Ericson, Real-time collision detection. Crc Press, 2004. 





[27] R. Rajamani, Vehicle dynamics and control. Springer Science & Business Media, 2011. 





[28] P. Polack, F. Altche, B. d’Andr ´ ea Novel, and A. de La Fortelle, “The ´ kinematic bicycle model: A consistent model for planning feasible trajectories for autonomous vehicles?” in 2017 IEEE intelligent vehicles symposium (IV). IEEE, 2017, pp. 812–818. 





[29] S. Gottschalk, M. C. Lin, and D. Manocha, “OBBTree: A hierarchical structure for rapid interference detection,” in Proceedings of the $2 3 r d$ annual conference on Computer graphics and interactive techniques, 1996, pp. 171–180. 





[30] J. Xu, P. Hu, and B. Alrifaee, “SigmaRL: A sample-efficient and generalizable multi-agent reinforcement learning framework for motion planning,” in 2024 IEEE 27th International Conference on Intelligent Transportation Systems (ITSC), in press. IEEE, 2024. 





[31] J. Xu, O. Sobhy, and B. Alrifaee, “XP-MARL: Auxiliary prioritization in multi-agent reinforcement learning to address non-stationarity,” arXiv preprint, 2024. 





[32] S. Diamond and S. Boyd, “CVXPY: A Python-embedded modeling language for convex optimization,” Journal of Machine Learning Research, vol. 17, no. 83, pp. 1–5, 2016. 

