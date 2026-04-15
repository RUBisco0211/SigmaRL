# High-Order Control Barrier Functions: Insights and a Truncated Taylor-Based Formulation

Jianye $\mathrm { { X u ^ { 1 } \mathbb { P } } }$ , Student Member, IEEE, Bassam Alrifaee2 , Senior Member, IEEE 

Abstract— We examine the complexity of the standard High-Order Control Barrier Function (HOCBF) approach and propose a truncated Taylor-based approach that reduces design parameters. First, we derive the explicit inequality condition for the HOCBF approach and show that the corresponding equality condition sets a lower bound on the barrier function value that regulates its decay rate. Next, we present our Truncated Taylor CBF (TTCBF), which uses a truncated Taylor series to approximate the discrete-time CBF condition. While the standard HOCBF approach requires multiple class $\kappa$ functions, leading to more design parameters as the constraint’s relative degree increases, our TTCBF approach requires only one. We support our theoretical findings in numerical collisionavoidance experiments and show that our approach ensures safety while reducing design complexity. 

# I. INTRODUCTION

Control Barrier Functions (CBFs) have received increasing attention in recent years as a way to ensure safety in dynamical systems [1]. The main idea behind CBFs is to guarantee that the system state stays within a predefined safe set by enforcing suitable constraints on the control inputs. By maintaining the forward invariance of this safe set, the system remains safe. 

A key component in CBFs is class $\kappa$ functions, which regulate how safety constraints are enforced. Specifically, they control the decay rate of the CBF value. The tuning of these functions strongly affects the balance between conservatism and aggression. A conservative choice can overly restrict system performance and impair the feasibility of the optimization problem, whereas an aggressive choice can allow excessive risk and potentially compromise safety [2]. The example in Fig. 1 illustrates how different class $\kappa$ functions influence a robot’s obstacle-avoidance behavior, ranging from slow and conservative to rapid and aggressive maneuvers. Note that conservatism in CBFs can also caused by factors like uncertainty handling [3] and overapproximation of the safe set [4], which are out of our scope. 

The standard CBF approaches cannot handle constraints with high relative degrees. The relative degree of a constraint is the number of times one must differentiate it along the system dynamics before the control input appears. Early solutions include a backstepping-based approach [5] and exponential CBFs with a virtual input-output linearization 

This research was supported by the Bundesministerium fur Digitales und ¨ Verkehr (German Federal Ministry for Digital and Transport) within the project “Harmonizing Mobility” (grant number 19FS2035A). 

1The author is with the Department of Computer Science, RWTH Aachen University, Germany, xu@embedded.rwth-aachen.de 

2The author is with the Department of Aerospace Engineering, University of the Bundeswehr Munich, Germany, bassam.alrifaee@unibw.de 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-15/2a1ca23f-107c-4ddc-83d7-f794bac18556/bf3bb644762d41137f6e55af6a2435c3e5419f47feba83052d6e5fa8cd4b1966.jpg)



Fig. 1: An obstacle-avoidance example with three different class $\kappa$ functions: conservative, moderate, and aggressive. Footprints in circles; trajectories in solid lines.


method [6]. Subsequently, the High-Order CBF (HOCBF) approach introduced in [7] provided a systematic way to extend standard CBF formulations by chaining high-order derivatives. However, it uses multiple class $\kappa$ functions, equaling the relative degree of the candidate CBF. This complicates control design, particularly for constraints with high relative degrees, since each class $\kappa$ function requires tuning. Various studies have been proposed to reduce the effort in the design of the class $\kappa$ functions in HOCBFs, including penalty methods [7], adaptive CBF [8], [9], and learning-based class $\kappa$ functions [10], [11]. 

In this work, we offer new insights into the HOCBF approach from [7] by deriving the explicit form of its safety condition and showing the imposed lower bound on the CBF value. This bound provides insights into the resulting controller and can serve to guide the tuning of class $\kappa$ functions. Furthermore, we propose an alternative approach to handle constraints with high relative degrees, termed Truncated Taylor CBF (TTCBF). It simplifies control design by using only one class $\kappa$ function. 

Section II reviews CBFs and HOCBFs. Section III presents the main results, including the analytical lower bound imposed by HOCBFs and our approach. Section IV validates the main results numerically. Section V discusses our approach, and Sec. VI concludes. 

# II. PRELIMINARIES

In this section, we briefly revisit CBFs and HOCBFs for continuous-time and discrete-time systems. 

We consider an input-affine control system given by 

$$
\dot {\boldsymbol {x}} = f (\boldsymbol {x}) + g (\boldsymbol {x}) \boldsymbol {u}, \tag {1}
$$

where the system state $\pmb { x } \in \mathcal { X } \subset \mathbb { R } ^ { n }$ and the control input $\pmb { u } \in \mathcal { U } \subseteq \mathbb { R } ^ { m }$ . Functions $f : \mathbb { R } ^ { n }  \mathbb { R } ^ { n }$ and $g : \mathbb { R } ^ { n }  \mathbb { R } ^ { n \times m }$ are globally Lipschitz. 

Definition 1 (Class $\kappa$ functions). A continuous function α : $[ 0 , a ) \to [ 0 , \infty )$ (with $a > 0$ ) is said to belong to class $\kappa$ if it is strictly increasing and $\alpha ( 0 ) = 0$ . 

Definition 2 (Forward invariant set). A set $C \subset { \mathcal { X } }$ is forward invariant for system (1) if its solution starting at any ${ \mathbf { } } x ( t _ { 0 } ) \in$ $C$ satisfies ${ \pmb x } ( t ) \in C , \forall t \geq t _ { 0 }$ . 

For a continuously differentiable function $h : \mathcal { X }  \mathbb { R }$ , let 

$$
C = \{\boldsymbol {x} \in \mathcal {X}: h (\boldsymbol {x}) \geq 0 \}. \tag {2}
$$

Definition 3 (CBFs [12]). Given a set $C$ as in (2), a continuously differentiable function $h ( \pmb { x } ) : \mathcal { X }  \mathbb { R }$ is a candidate CBF for system (1) on $C$ if there exists a class $\kappa$ function $\alpha$ such that 

$$
\sup  _ {\boldsymbol {u} \in \mathcal {U}} \left[ \dot {h} (\boldsymbol {x}, \boldsymbol {u}) + \alpha (h (\boldsymbol {x})) \right] \geq 0, \quad \forall \boldsymbol {x} \in C, \tag {3}
$$

where $\dot { h } ( { \pmb x } , { \pmb u } )$ denotes the time derivative of $h$ and can also be expressed as $L _ { f } h ( { \pmb x } ) + L _ { g } h ( { \pmb x } ) { \pmb u }$ , with $L _ { f } h$ and $L _ { g } h$ denoting the Lie derivatives of $h$ along $f$ and $g$ , respectively. 

Now, consider system (1) in the discrete-time domain given by 

$$
\boldsymbol {x} _ {k + 1} = f (\boldsymbol {x} _ {k}) + g (\boldsymbol {x} _ {k}) \boldsymbol {u} _ {k}, \tag {4}
$$

where the subscript $k \in \mathbb N$ denotes the time step. 

Definition 4 (Discrete-Time CBFs [13]). Given a set $C$ as in (2), a continuous function $h ( \pmb { x } ) : \mathcal { X }  \mathbb { R }$ is a candidate discrete-time CBF for system (4) on $C$ if there exists a class $\kappa$ function $\alpha$ satisfying $\alpha ( z ) \leq z$ such that 

$$
\sup  _ {\boldsymbol {u} \in \mathcal {U}} \left[ \Delta h \left(\boldsymbol {x} _ {k}, \boldsymbol {u} _ {k}\right) + \alpha \left(h \left(\boldsymbol {x} _ {k}\right)\right) \right] \geq 0, \quad \forall \boldsymbol {x} _ {k} \in C. \tag {5}
$$

One common variant is the discrete-time exponential CBFs [13], in which a linear class $\kappa$ function is used, i.e., $\alpha ( h ( \pmb { x } _ { k } ) ) = \lambda h ( \pmb { x } _ { k } )$ , where $\lambda \in ( 0 , 1 ]$ denotes its parameter. Substituting $\Delta h ( \boldsymbol { x } _ { k } , \boldsymbol { u } _ { k } )$ in (5) with forward difference $h ( \pmb { x } _ { k + 1 } ) - h ( \pmb { x } _ { k } )$ and rearranging yields 

$$
\sup  _ {\boldsymbol {u} \in \mathcal {U}} \left[ h (\boldsymbol {x} _ {k + 1}) - (1 - \lambda) h (\boldsymbol {x} _ {k}) \right] \geq 0, \quad \forall \boldsymbol {x} _ {k} \in C. \tag {6}
$$

At each time step $k \in \mathbb { N }$ , the control input enforces $h ( \pmb { x } _ { k + 1 } ) \geq ( 1 - \lambda ) h ( \pmb { x } _ { k } )$ , rendering $h ( \pmb { x } _ { k } )$ may decrease by at most the factor $1 - \lambda$ . Applying this recursively yields the imposed lower bound 

$$
h \left(\boldsymbol {x} _ {k}\right) \geq (1 - \lambda) ^ {k - k _ {0}} h \left(\boldsymbol {x} _ {k _ {0}}\right), \quad \forall k \geq k _ {0}, \tag {7}
$$

where $\pmb { x } _ { k _ { 0 } } \in \mathcal { X }$ denotes the initial state at time step $k _ { 0 }$ . This lower bound is an exponential function in time step $k$ and hence the name exponential CBFs. It becomes 

$$
h (\boldsymbol {x}) \geq e ^ {- \lambda (t - t _ {0})} h _ {\boldsymbol {x} _ {t _ {0}}}, \quad \forall t \geq t _ {0} \tag {8}
$$

in the continuous-time domain, where $\pmb { x } _ { t _ { 0 } } \in \mathcal { X }$ denotes the initial state at time $t _ { 0 }$ , and $\lambda > 0$ (instead of $\lambda \in ( 0 , 1 ] )$ . 

In practice, CBFs often have high relative degrees. The relative degree of a function is the number of times one must differentiate it along the system dynamics until the control input appears explicitly. 

Definition 5 (Relative degree). A continuously differentiable function $h : \mathcal { X }  \mathbb { R }$ is said to have relative degree $r \in \mathbb N$ with respect to system (1) or (4) if $\forall x \in \mathcal { X }$ , $L _ { g } L _ { f } ^ { i } h ( { \pmb x } ) =$ $0 , \forall i \in \{ 0 , 1 , \ldots , r - 2 \}$ and $L _ { g } L _ { f } ^ { r - 1 } h ( { \pmb x } ) \neq 0$ . 

Since we use CBFs to define constraints, we use the relative degrees of CBFs and relative degrees of constraints interchangeably. If $r > 1$ , the standard CBF condition (3) or (5) fails to capture the control input $\textbf { \em u }$ (because $L _ { g } h ( { \pmb x } ) =$ 0). Study [7] proposed HOCBFs to address this limitation. Recursively, define auxiliary functions $\Psi _ { 0 } ( { \pmb x } ) : = h ( { \pmb x } )$ and 

$$
\Psi_ {i} (\boldsymbol {x}) := \dot {\Psi} _ {i - 1} (\boldsymbol {x}) + \alpha_ {i} \left(\Psi_ {i - 1} (\boldsymbol {x})\right), \quad \forall i = \{1, \dots , r \}, \tag {9}
$$

where each $\alpha _ { i }$ is a differentiable class $\kappa$ function. Let 

$$
C _ {i} := \left\{\boldsymbol {x} \in \mathcal {X}: \Psi_ {i - 1} (\boldsymbol {x}) \geq 0 \right\}, \quad \forall i \in \{1, \dots , r \}. \tag {10}
$$

Note that by construction, $C _ { 1 }$ is the original set where we want to ensure forward invariance. 

Definition 6 (HOCBFs [7]). Given sets $C _ { i }$ as in (10), a continuously differentiable function $h : \mathcal { X } \ \to \ \mathbb { R }$ is a candidate HOCBF with relative degree $r$ for system (1) if there exist continuously differentiable class $\kappa$ functions $\alpha _ { i } , \forall i \in \{ 1 , . . . , r \}$ , such that 

$$
\sup  _ {\boldsymbol {u} \in \mathcal {U}} \left[ \Psi_ {r} (\boldsymbol {x}, \boldsymbol {u}) \right] \geq 0, \quad \forall \boldsymbol {x} \in \bigcap_ {i = 1} ^ {r} C _ {i}. \tag {11}
$$

As shown in [7], enforcing (11) inductively ensures $\Psi _ { i } \geq$ $0 , \forall i \in \{ 0 , \ldots , r - 1 \}$ , thereby rendering set $C _ { 1 }$ (and $\cap _ { i = 2 } ^ { r } C _ { i } )$ forward invariant. A common variant of HOCBFs is exponential $H O C B F s$ , which adopt linear class $\kappa$ functions when defining auxiliary functions (9), i.e., $\alpha _ { i } \big ( \Psi _ { i - 1 } ( { \pmb x } ) \big ) =$ $\lambda _ { i } \Psi _ { i - 1 } ( { \pmb x } )$ , where $\lambda _ { i } > 0$ denotes the parameter of the linear class $\kappa$ function $\alpha _ { i }$ . 

# III. MAIN RESULTS

In Sec. III-A, we provide insights into the HOCBF approach introduced by [7]. In Sec. III-B, we propose an alternative approach based on the truncated Taylor series to handle CBFs with high relative degrees. 

# A. Insights Into HOCBFs

Our insights include the explicit form of the condition imposed by exponential HOCBFs and the analytical lower bound imposed by this condition. 

Lemma 1. Consider an HOCBF $h$ in Definition 6 with relative degree $r$ . For each $i \in \{ 1 , \ldots , r \}$ , the corresponding auxiliary function defined in (9) with a linear class $\kappa$ function can be flattened and equivalently expressed as 

$$
\Psi_ {i} (\boldsymbol {x}) \equiv \sum_ {j = 0} ^ {i} e _ {i - j} \left(\lambda_ {1}, \dots , \lambda_ {i}\right) h ^ {(j)} (\boldsymbol {x}). \tag {12}
$$

Here, $\begin{array} { r } { e _ { i - j } \left( \lambda _ { 1 } , \dots , \lambda _ { i } \right) : = \sum \underset { | I | = j - i } { I \subseteq \{ 1 , \dots , i \} } \prod _ { k \in I } \lambda _ { k } } \end{array}$ |I|=i−j denotes an elementary symmetric polynomial1, $\lambda _ { 1 } , \ldots , \lambda _ { r }$ denote the 

$^ { 1 } \mathrm { E . g . }$ , if $i = 3$ , then $e _ { 0 } ( \lambda _ { 1 } , \lambda _ { 2 } , \lambda _ { 3 } ) = 1 , e _ { 1 } ( \lambda _ { 1 } , \lambda _ { 2 } , \lambda _ { 3 } ) = \lambda _ { 1 } + \lambda _ { 2 } +$ $e _ { 0 } ( \lambda _ { 1 } , \lambda _ { 2 } , \lambda _ { 3 } ) = 1$ $\lambda _ { 3 } , e _ { 2 } ( \lambda _ { 1 } , \lambda _ { 2 } , \lambda _ { 3 } ) = \lambda _ { 1 } \lambda _ { 2 } + \lambda _ { 1 } \lambda _ { 3 } + \lambda _ { 2 } \lambda _ { 3 } , e _ { 3 } ( \lambda _ { 1 } , \lambda _ { 2 } , \lambda _ { 3 } ) = \lambda _ { 1 } \lambda _ { 2 } \lambda _ { 3 } .$ $e _ { 3 } ( \lambda _ { 1 } , \lambda _ { 2 } , \lambda _ { 3 } ) = \lambda _ { 1 } \lambda _ { 2 } \lambda _ { 3 }$ Note that $e _ { 0 } ( \cdot ) = 1$ by convention. 

parameters of the linear class $\kappa$ functions, and $\it { h ^ { ( i ) } }$ denotes the ith time derivative of the candidate CBF $h$ . 

Proof. We proceed by induction on $i$ 

Base Case $( i ~ = ~ 1 )$ : By definition, $\Psi _ { 1 } ( { \pmb x } ) : = \dot { h } ( { \pmb x } ) +$ $\lambda _ { 1 } h ( \pmb { x } )$ . Since $e _ { 1 } ( \lambda _ { 1 } ) ~ = ~ \lambda _ { 1 }$ and $e _ { 0 } ( \lambda _ { 1 } ) { \mathrm { ~ \dot { = ~ 1 } ~ } }$ , we have $\Psi _ { 1 } \left( \pmb { x } \right) \equiv e _ { 1 } ( \lambda _ { 1 } ) h \left( \pmb { x } \right) + e _ { 0 } ( \lambda _ { 1 } ) \dot { h } ( \pmb { x } )$ 

Inductive Step: Assume for some $i \in \{ 1 , \ldots , r - 1 \}$ , 

$$
\Psi_ {i} (\boldsymbol {x}) \equiv \sum_ {j = 0} ^ {i} e _ {i - j} \left(\lambda_ {1}, \dots , \lambda_ {i}\right) h ^ {(j)} (\boldsymbol {x}), \tag {13}
$$

and we need to show 

$$
\Psi_ {i + 1} (\boldsymbol {x}) \equiv \sum_ {j = 0} ^ {i + 1} e _ {i + 1 - j} \left(\lambda_ {1}, \dots , \lambda_ {i + 1}\right) h ^ {(j)} (\boldsymbol {x}). \tag {14}
$$

The time derivative of (13) is given by $\begin{array} { r l } { \dot { \Psi } _ { i } ( \pmb { x } ) } & { { } \equiv } \end{array}$ $\begin{array} { r } { \sum _ { j = 0 } ^ { i } e _ { i - j } \left( \lambda _ { 1 } , \ldots , \lambda _ { i } \right) h ^ { ( j + 1 ) } ( { \pmb x } ) } \end{array}$ . By definition in (9), $\bar { \Psi _ { i + 1 } } ( \pmb { x } ) : = \dot { \Psi } _ { i } ( \pmb { x } ) + \lambda _ { i + 1 } \Psi _ { i } ( \pmb { x } )$ . Substituting $\dot { \Psi } _ { i }$ and $\Psi _ { i }$ , we obtain $\begin{array} { r } { \Psi _ { i + 1 } \big ( \pmb { x } \big ) \equiv \sum _ { j = 0 } ^ { i } e _ { i - j } \big ( \lambda _ { 1 } , \ldots , \lambda _ { i } \big ) h ^ { ( j + 1 ) } \big ( \pmb { x } \big ) + } \end{array}$ $\begin{array} { r } { \lambda _ { i + 1 } \sum _ { j = 0 } ^ { i } e _ { i - j } \left( \lambda _ { 1 } , \ldots , \overset { \sim } { \lambda _ { i } } \right) h ^ { ( j ) } ( \pmb { x } ) } \end{array}$ . Consider the property of the elementary symmetric polynomial that $\begin{array} { r l r l r } { e _ { k } \left( \lambda _ { 1 } , \dots , \lambda _ { i + 1 } \right) } & { { } } & { = { } } & { { } } & { e _ { k } \left( \lambda _ { 1 } , \dots , \lambda _ { i } \right) \quad + } \end{array}$ $\begin{array} { r l } { e _ { k } \left( \lambda _ { 1 } , \ldots , \lambda _ { i } \right) } & { { } + } \end{array}$ $\lambda _ { i + 1 } e _ { k - 1 } \big ( \lambda _ { 1 } , \ldots , \lambda _ { i } \big ) , \forall k \in \{ 0 , \ldots , i + 1 \}$ , and note the convention $e _ { i + 1 } ( \lambda _ { 1 } , \ldots , \lambda _ { i } ) = 0$ and $e _ { - 1 } ( \lambda _ { 1 } , . . . , \lambda _ { i } ) = 0$ , one easily obtains (14). □ 

Theorem 1. Let $h$ be an HOCBF as in Definition 6 with relative degree $r$ and associated sets $C _ { i }$ as in (10), and let class $\kappa$ functions in (9) be linear with parameters $\lambda _ { i } , \forall i \in$ $\{ 1 , \ldots , r \}$ . Any Lipschitz continuous controller satisfying 

$$
\sup  _ {\boldsymbol {u} \in \mathcal {U}} \left[ \sum_ {j = 0} ^ {r} e _ {r - j} \left(\lambda_ {1}, \dots , \lambda_ {r}\right) h ^ {(j)} (\boldsymbol {x}) \right] \geq 0 \tag {15}
$$

renders $C _ { 1 }$ (and $\cap _ { i = 2 } ^ { r } C _ { i } )$ forward invariant for system (1). 

Proof. Following Lemma 1, the left side of (15) is equivalent to the last auxiliary function $\Psi _ { r } ( { \pmb x } , { \pmb u } )$ in HOCBFs. Satisfying (15) is equivalent to satisfying the standard HOCBF condition (11). The remaining proof follows [7, Thm. 5]. 

Corollary 1. Consider an exponential HOCBF $h ( t )$ with relative degree $r$ and initial conditions 

$$
\boldsymbol {h} _ {\text {i n i t}} := \left[ h _ {t _ {0}}, \dot {h} _ {t _ {0}}, \dots , h _ {t _ {0}} ^ {(r - 1)} \right] ^ {\top} \in \mathbb {R} ^ {r} \tag {16}
$$

at an initial time $t _ { 0 }$ . The lower bound of $h ( t )$ , denoted as $h _ { \mathrm { l b } } ( t )$ , imposed by the exponential HOCBF is given by 

$$
h _ {\mathrm {l b}} (t) := \sum_ {i = 1} ^ {r} c _ {i} e ^ {- \lambda_ {i} (t - t _ {0})}, \quad \forall t \geq t _ {0}, \tag {17}
$$

subject to the same initial conditions $h _ { \mathrm { i n i t } }$ in (16). The coefficient $\begin{array} { r l r } {  { \boldsymbol { c } } } & { { } : = \left[ c _ { 1 } , \dots , c _ { r } \right] ^ { \top } } & { \in \mathbb { R } ^ { r } } \end{array}$ is deterministically determined by solving 

$$
M _ {\lambda} \boldsymbol {c} = \boldsymbol {h} _ {\text {i n i t}}, \tag {18}
$$

where 

$$
M _ {\lambda} = \left[ \begin{array}{c c c c} (- \lambda_ {1}) ^ {0} & (- \lambda_ {2}) ^ {0} & \dots & (- \lambda_ {r}) ^ {0} \\ (- \lambda_ {1}) ^ {1} & (- \lambda_ {2}) ^ {1} & \dots & (- \lambda_ {r}) ^ {1} \\ \vdots & \vdots & \ddots & \vdots \\ (- \lambda_ {1}) ^ {r - 1} & (- \lambda_ {2}) ^ {r - 1} & \dots & (- \lambda_ {r}) ^ {r - 1} \end{array} \right] \in \mathbb {R} ^ {r \times r} \tag {19}
$$

is a Vandermonde-like matrix, and $\lambda _ { 1 } , \ldots , \lambda _ { r }$ are the parameters of the linear class $\kappa$ functions. 

Proof. Following Theorem 1, the condition imposed by the exponential HOCBF is equivalent to (15). Observe that the equality form of (15), i.e., 

$$
\sum_ {j = 0} ^ {r} e _ {r - j} \left(\lambda_ {1}, \dots , \lambda_ {r}\right) h ^ {(j)} (t) = 0, \tag {20}
$$

is a homogeneous linear differential equation, with characteristic equation $\begin{array} { r } { \sum _ { j = 0 } ^ { r } e _ { r - j } \left( \lambda _ { 1 } , \ldots , \bar { \lambda _ { r } } \right) \lambda ^ { j } \ = \ 0 } \end{array}$ . Given the property of the elementary symmetric polynomial, this characteristic equation can be equivalently expressed as $\begin{array} { r } { \prod _ { i } ^ { r } ( \lambda { + } \lambda _ { i } ) = 0 } \end{array}$ , with roots $\lambda _ { 1 } , \ldots , \lambda _ { r }$ .2 Importantly, observe that (17) is the unique solution for (20), i.e., $L _ { D } h _ { \mathrm { l b } } ( t ) = 0$ holds, where $L _ { D }$ denotes a linear polynomial differential operator to an arbitrary continuously differentiable function $y ( t )$ as $\begin{array} { r } { L _ { D } y ( t ) : = \dot { \sum } _ { j = 0 } ^ { r } e _ { r - j } \big ( \dot { \lambda _ { 1 } } , \dot { \bf \Xi } , \dot { \bf \Xi } , \lambda _ { r } \big ) y ^ { ( j ) } ( t ) } \end{array}$ . Then, (15) is equivalent to $L _ { D } ^ { \mathrm { ~ ~ } } h ( t ) \geq 0$ . Introduce an auxiliary function $h _ { \mathrm { a u x } } ( t ) : = h ( t ) - h _ { \mathrm { l b } } ( t )$ . By linearity of $L _ { D }$ , $L _ { D } h _ { \mathrm { a u x } } ( t ) = L _ { D } h ( t ) - L _ { D } h _ { \mathrm { l b } } ( t ) \geq 0 , \forall t \geq t _ { 0 }$ . Moreover, $h _ { \mathrm { a u x } } ( t _ { 0 } ) = h ( t _ { 0 } ) - h _ { \mathrm { l b } } ( t _ { 0 } ) = 0$ since $h$ and $h _ { \mathrm { l b } }$ have the same initial condition. By the maximum principle, it follows that $h _ { \mathrm { a u x } } ( t ) \geq 0 , \forall t \geq t _ { 0 }$ , i.e., $h ( t ) \geq h _ { \mathrm { l b } } ( t ) , \forall t \geq t _ { 0 }$ . We conclude that $h _ { \mathrm { l b } } ( t )$ in (17) is the lower bound of $h ( t )$ . 

Remark 1. Following Corollary 1, the lower bound imposed by exponential HOCBFs depends not only on the parameters of the linear class $\kappa$ functions but also on the initial conditions of the candidate CBF and its time derivatives. On the other hand, HOCBFs require that the initial value of each auxiliary function in (9) (except for the last one, $\Psi _ { r } )$ ) be non-negative [7]. To realize this, one can recursively show that sufficient (but not necessary) conditions are $\lambda _ { i } \geq$ $- h ^ { ( i ) } / h ^ { ( i - 1 ) } , \forall i \in \left\{ 1 , \dots , r - 1 \right\}$ , and $\lambda _ { r } > 0$ . Therefore, the selection of the parameters is restricted by these initial conditions. These couplings can complicate control design, which motivates our approach presented next. 

# B. Truncated Taylor CBFs

This section presents our TTCBF approach to handle constraints with high relative degrees while requiring only one class $\kappa$ function. 

Consider a CBF with relative degree $r$ . The fundamental discrete-time CBF condition that we need to guarantee is (5), i.e., $\Delta h ( { \pmb x } _ { k } , { \pmb u } _ { k } ) \ : + \ : \alpha \big ( h ( { \pmb x } _ { k } ) \big ) \ : \ge \ : 0 , \forall { \pmb x } _ { k } \in \ : { \cal C }$ . We approximate $\dot { \Delta } h ( \boldsymbol { x } _ { k } , \dot { \boldsymbol { u } _ { k } } )$ with a truncated Taylor series with up to the rth time derivative of $h$ , i.e., $\Delta h ( \boldsymbol { x } _ { k } , \boldsymbol { u } _ { k } ) \ \approx$ 

2Take $r = 2$ as an example: $\begin{array} { r } { \sum _ { j = 0 } ^ { 2 } e _ { 2 - j } \left( \lambda _ { 1 } , \lambda _ { 2 } \right) \lambda ^ { j } = \lambda ^ { 2 } + ( \lambda _ { 1 } + } \end{array}$ $\lambda _ { 2 } ) \lambda + \lambda _ { 1 } \lambda _ { 2 } = ( \lambda + \lambda _ { 1 } ) ( \lambda + \lambda _ { 2 } )$ , with roots $\lambda = - \lambda _ { 1 }$ and $\lambda = - \lambda _ { 2 }$ . 

$\begin{array} { r } { \Delta t \dot { h } ( { \pmb x } _ { k } ) + \frac { 1 } { 2 } \Delta t ^ { 2 } \ddot { h } ( { \pmb x } _ { k } ) + \dots + \frac { 1 } { r ! } \Delta t ^ { r } h ^ { ( r ) } ( { \pmb x } _ { k } , { \pmb u } _ { k } ) } \end{array}$ . Here, the $r$ th derivative $h ^ { ( r ) } ( \pmb { x } _ { k } , \pmb { u } _ { k } )$ captures the control input. 

Definition 7 (Truncated Taylor CBFs). Given a set $C$ as in (2). An $( r + 1 ) \mathrm { t h }$ continuously differentiable function $h :$ $\mathcal { X }  \mathbb { R }$ is a candidate Truncated Taylor CBF (TTCBF) with relative degree $r$ for system (4) if there exists a class $\kappa$ function $\alpha$ satisfying $\alpha ( z ) \leq z$ such that $\forall x _ { k } \in C$ , 

$$
\begin{array}{l} \sup  _ {\boldsymbol {u} _ {k} \in \mathcal {U}} \left[ \Delta t \dot {h} \left(\boldsymbol {x} _ {k}\right) + \dots + \frac {1}{r !} \Delta t ^ {r} h ^ {(r)} \left(\boldsymbol {x} _ {k}, \boldsymbol {u} _ {k}\right) + \right. \tag {21} \\ \left. \alpha \left(h \left(\boldsymbol {x} _ {k}\right)\right) \right] \geq \gamma \Delta t ^ {r + 1}, \\ \end{array}
$$

Here, $\gamma$ is a design parameter that satisfies $\begin{array} { r } { \gamma \geq \frac { \Gamma _ { r + 1 } } { ( r + 1 ) ! } } \end{array}$ r+1(r+1)! , and we assume that the $( r + 1 )$ th time derivative of $h$ is uniformly bounded over $C$ , i.e., there exists a constant $\Gamma _ { r + 1 } > 0$ such that $\big | h ^ { ( r + 1 ) } ( { \pmb x } ) \big | \le \Gamma _ { r + 1 } , \forall { \pmb x } \in C$ . 

Theorem 2. Given a TTCBF in Definition 7 with associated set $C$ from (2), any Lipschitz continuous controller satisfying (21) renders $C$ forward invariant for system (4). 

Proof. Because $h$ is $( r + 1 ) \mathrm { t h }$ continuously differentiable, Taylor’s theorem guarantees the existence of a point $\boldsymbol { \xi }$ between $\scriptstyle { \mathbf { { \mathit { x } } } } _ { k }$ and $\scriptstyle { \pmb { x } } _ { k + 1 }$ such that $h ( \pmb { x } _ { k + 1 } ) ~ = ~ h ( \pmb { x } _ { k } ) ~ +$ $\begin{array} { r } { \Delta t { \dot { h } } ( { \pmb x } _ { k } ) + \dots + \frac { 1 } { r ! } \Delta t ^ { r } h ^ { ( r ) } ( { \pmb x } _ { k } , { \pmb u } _ { k } ) + R _ { r + 1 } } \end{array}$ , with the remainder $\begin{array} { r } { R _ { r + 1 } : = \frac { 1 } { ( r + 1 ) ! } \Delta t ^ { r + 1 } h ^ { ( r + 1 ) } ( \pmb { \xi } ) } \end{array}$ . Since the $h ^ { ( r + 1 ) }$ is bounded by $\Gamma _ { r + 1 }$ , it follows that $\begin{array} { r } { | R _ { r + 1 } | \leq \frac { \Gamma _ { r + 1 } } { ( r + 1 ) ! } \Delta t ^ { r + 1 } } \end{array}$ Γr+1 . Thus, we have $h ( \pmb { x } _ { k + 1 } ) \ \geq \ h ( \pmb { x } _ { k } ) + \Delta t \dot { h } ( \pmb { x } _ { k } ) + \cdot \cdot \cdot +$ $\begin{array} { r } { \frac { 1 } { r ! } \Delta t ^ { r } h ^ { ( r ) } ( { \pmb x } _ { k } ) - \frac { \Gamma _ { r + 1 } } { ( r + 1 ) ! } \Delta t ^ { r + 1 } } \end{array}$ . Given (21), one easily obtains 

$$
h \left(\boldsymbol {x} _ {k + 1}\right) \geq h \left(\boldsymbol {x} _ {k}\right) - \alpha \left(h \left(\boldsymbol {x} _ {k}\right)\right) + \left(\gamma - \frac {\Gamma_ {r + 1}}{(r + 1) !}\right) \Delta t ^ {r + 1}. \tag {22}
$$

Provided that γ ≥ $\begin{array} { r } { \gamma \ge \frac { \Gamma _ { r + 1 } } { ( r + 1 ) ! } } \end{array}$ Γr+1 , we have $h ( \pmb { x } _ { k + 1 } ) \geq h ( \pmb { x } _ { k } ) -$ $\alpha \big ( h ( \pmb { x } _ { k } ) \big )$ , i.e., $\Delta h ( \dot { \mathbf { x } } _ { k } ) + \alpha \big ( h ( \mathbf { x } _ { k } ) \big ) \geq 0$ , rendering (21) to be a sufficient condition of the discrete-time CBF condition (5). The remaining proof follows [14, Thm. 1]. □ 

The design parameter $\gamma$ in (21) considers the Taylor series truncation error. We will discuss how to handle it in Sec. V. 

# IV. NUMERICAL EXPERIMENTS

We revisit the collision-avoidance example in Fig. 1. In Sec. IV-A, we formulate the Quadratic Program (QP) problem using both the standard HOCBF approach and our TTCBF approach. We present the experimental results of the standard HOCBF approach in Sec. IV-B and those of our approach in Sec. IV-C. Code reproducing the experimental results is available at our open-source repository3 [15]. 

# A. Quadratic Program Formulation

To facilitate the computation of time derivatives, we select the square of the distance between the robot and the obstacle as the candidate CBF, i.e., 

$$
h (\boldsymbol {x} (t)) = (x (t) - x _ {\mathrm {o b s}}) ^ {2} + (y (t) - y _ {\mathrm {o b s}}) ^ {2} - (r + r _ {\mathrm {o b s}}) ^ {2}, \tag {23}
$$


TABLE I: Parameters used in the experiments.


<table><tr><td>Parameters</td><td>Values</td></tr><tr><td>Robot and obstacle radii r, robs</td><td>1 m, 2 m</td></tr><tr><td>Obstacle position (xobs, yobs)</td><td>(0 m, -3.1 m)</td></tr><tr><td>Robot initial position</td><td>(-10 m, 0 m)</td></tr><tr><td>Robot initial speed, acceleration</td><td>10 m/s, 0 m/s2</td></tr><tr><td>Reference yref, vx, ref, vy, ref</td><td>0 m, 10 m/s, 0 m/s</td></tr><tr><td>Penalty parameters pvx, pvy, py</td><td>1, 1, 1000</td></tr><tr><td>Admissible control input umin, umax</td><td>±1000 m/s2</td></tr><tr><td>Sampling period Δt, simulation duration</td><td>0.01 s, 2 s</td></tr><tr><td>Term γΔtr+1 in (21)</td><td>≈ 0</td></tr></table>

where $\pmb { x } ( t ) : = \left( x ( t ) , y ( t ) \right) ^ { \top } \in \mathbb { R } ^ { 2 }$ and $( x _ { \mathrm { o b s } } , y _ { \mathrm { o b s } } ) \in \mathbb { R } ^ { 2 }$ denote the positions of the robot and the obstacle, respectively. The radii of the robot and the obstacle are given by $r > 0$ and $r _ { \mathrm { o b s } } > 0$ , respectively. Clearly, if $h ( { \pmb x } ( t ) ) \geq 0$ for all $t > 0$ , the robot remains safe. We model the robot dynamics as a double integrator and choose the accelerations along the $x$ - and $y$ -directions as the control inputs, denoted by $\bar { \mathbf { \ b { u } } } ( t ) : = \left( \ b { u } _ { x } ( t ) , \ b { u } _ { y } ( t ) \right) ^ { \top }$ . Therefore, the candidate CBF in (23) has relative degree two. Since the obstacle position is constant, the first- and second-time derivatives of $h$ (omitting the explicit time dependence for brevity) are 

$$
\dot {h} = 2 \left(x - x _ {\mathrm {o b s}}\right) \dot {x} + 2 \left(y - y _ {\mathrm {o b s}}\right) \dot {y},
$$

$$
\ddot {h} = 2 \left(\dot {x} ^ {2} + \dot {y} ^ {2} + \left(x - x _ {\mathrm {o b s}}\right) u _ {x} + \left(y - y _ {\mathrm {o b s}}\right) u _ {y}\right).
$$

We conduct the experiments in simulation using discrete time. We use a sampling period of $\Delta t = 0 . 0 1$ s and set the duration of each simulation to 2 s. Let $k \in \mathbb N$ denote the current time step. We formulate the QP problem as 

$$
\begin{array}{l} \boldsymbol {u} _ {k} = \arg \min  _ {\boldsymbol {u} _ {k}} p _ {v _ {x}} \left(\hat {v} _ {x, k + 1} - v _ {x, \text {r e f}}\right) ^ {2} + \\ p _ {v _ {y}} \left(\hat {v} _ {y, k + 1} - v _ {y, \text {r e f}}\right) ^ {2} + p _ {y} \left(\hat {y} _ {k + 1} - y _ {\text {r e f}}\right) ^ {2}, \tag {24a} \\ \end{array}
$$

s.t. $\ddot { h } _ { k } + ( \lambda _ { 1 } + \lambda _ { 2 } ) \dot { h } _ { k } + \lambda _ { 1 } \lambda _ { 2 } h _ { k } \geq 0$ (see (12)) or 

$$
\Delta t \dot {h} _ {k} + \frac {1}{2} \Delta t ^ {2} \ddot {h} _ {k} + \lambda_ {1} h _ {k} \geq \gamma \Delta t ^ {3} (\text {s e e (2 1)}), \tag {24b}
$$

$$
\boldsymbol {u} _ {\min } \leq \boldsymbol {u} _ {k} \leq \boldsymbol {u} _ {\max }. \tag {24c}
$$

The cost function in (24a) comprises three terms. The first two terms penalize deviations from the reference speeds, $v _ { x , \mathrm { r e f } }$ along the $x$ -axis and $v _ { y , \mathrm { r e f } }$ along the $y$ -axis. The predicted next-step speed along the $x$ -axis is computed as $\hat { v } _ { x , k + 1 } ( u _ { x , k } ) \ = \ v _ { x , k } + \Delta t u _ { x , k }$ , under a zero-order hold assumption of the control input, where $v _ { x , k }$ denotes the current $x$ -speed. Similarly, $\hat { v } _ { y , k + 1 } ( u _ { y , k } ) = v _ { y , k } + \Delta t u _ { y , k }$ , and $v _ { y , \mathrm { r e f } }$ is the reference speed along the $y$ -axis. In addition, to encourage the robot to follow its reference path, which is a horizontal line defined by $y = y _ { \mathrm { r e f } }$ , we penalize the deviation from it. The predicted next-step $y$ -position is computed as $\begin{array} { r } { \hat { y } _ { k + 1 } ( u _ { y , k } ) \ = \ y _ { k } + \Delta t v _ { y , k } + \frac { 1 } { 2 } \Delta t ^ { 2 } u _ { y , k } } \end{array}$ . The penalty parameters are denoted by $p _ { v _ { x } }$ , $p _ { v _ { y } }$ , and $p _ { y }$ . Constraint (24b) is the CBF condition that ensures safety and is computed as (12) when using the standard HOCBF approach or as (21) 

when using our approach. We use linear class $\kappa$ functions for both approaches. Since the CBF (23) has relative degree two, the HOCBF approach requires two class $\kappa$ functions, with their parameters denoted by $\lambda _ { 1 }$ and $\lambda _ { 2 }$ . Our approach requires only one, with its parameter denoted by $\lambda _ { 1 }$ . For simplicity, we ignore the term $\gamma \Delta t ^ { 3 }$ in (24b) in the case of our approach. Constraint (24c) imposes admissible control input, which is set to sufficiently high since they are not the focus of our experiments. We solve (24) iteratively at each time step $k$ and apply the solution $\mathbf { \Delta } \mathbf { u } _ { k }$ . 

# B. First Experiment: HOCBF Approach

In this experiment, we apply the standard HOCBF approach and evaluate its performance under different parameter settings of the class $\kappa$ functions. We run simulations over a dense grid of parameter pairs, with $( \lambda _ { 1 } , \lambda _ { 2 } ) \in [ 2 . 1 , 1 0 ] \times$ [0.5, 10]. Note that we choose $\lambda _ { 1 , \mathrm { { m i n } } } ~ = ~ 2 . 1$ because the HOCBF approach requires $\lambda _ { 1 } \geq - \dot { h } _ { 0 } / h _ { 0 }$ , as discussed in Remark 1. From the given initial conditions (see Table I), we compute $- \dot { h } _ { 0 } / h _ { 0 } \overset { \mathbf { \cdot } } { \approx } 2 . 1 0$ . We empirically set the other three bounds, $\lambda _ { \mathrm { 1 , m a x } }$ , $\lambda _ { 2 , \mathrm { m i n } }$ , and $\lambda _ { \mathrm { { 2 , \mathrm { { m a x } } } } }$ , to be sufficiently low or high to cover a broad range of control behaviors. With our QP formulation in (24), the controller decelerates the robot when the CBF condition is active, especially in the $x$ -direction. Therefore, we use the mean $x$ -speed during each simulation as an indicator of the controller’s performance. Using the total mean speed yields similar results. 

One typical statement in CBFs is that a linear class $\kappa$ function with a larger parameter is less conservative, and vice versa [2]. However, our experimental results show that this does not necessarily hold in HOCBFs. Figure 2a shows the mean $x$ -speed for all parameter pairs. A higher mean speed indicates a less conservative controller. As observed, if $\lambda _ { 1 } ~ > ~ 3 . 0$ , the achieved mean $x$ -speed increases with $\lambda _ { 2 }$ ; otherwise, it remains unchanged regardless of $\lambda _ { 2 }$ . In contrast, if $\lambda _ { 2 } ~ < ~ 5 . 3$ , increasing $\lambda _ { 1 }$ reduces the achieved mean $x$ -speed, and vice versa. These observations can be explained by our analytical lower bound (17). Although the decay constant $\lambda _ { i }$ of each sum term matches the parameter of one of the linear class $\kappa$ functions, its coefficient $c _ { i }$ is jointly determined by all these parameters and the initial conditions (see (18)), preventing the direct application of the above statement in HOCBFs. 

Figure 2b shows the robot footprints for each parameter pair, using the same color coding as in Fig. 2a. Some parameter pairs cause the robot to fail to bypass the obstacle within the simulation duration, because the imposed lower bound on the CBF value is high, forcing a low decay rate of $h ( t )$ . Figure 2c illustrates this with two representative parameter pairs. The solid lines represent actual CBF values, and the dotted lines with circle markers represent analytical lower bounds computed by (17). Consider the most conservative parameter pair with $\lambda _ { 1 } ~ = ~ 1 0$ and $\lambda _ { 2 } ~ = ~ 0 . 5$ . Applying (17) with the coefficients $^ c$ obtained by solving (18) under the initial condition $t _ { 0 } ~ = ~ 0 \mathrm { s }$ (i.e., $\begin{array} { r } { \dot { h _ { \mathrm { i n i t } } } ~ = ~ [ \dot { h } _ { t _ { 0 } } , \dot { h } _ { t _ { 0 } } ] ^ { \top } ~ = ~ } \end{array}$ 

$[ 9 5 . 8 , - 2 0 0 ] ^ { \top } )$ yields 

$$
\boldsymbol {c} = M _ {\lambda} ^ {- 1} \boldsymbol {h} _ {\mathrm {i n i t}} = \left[ \begin{array}{c c} 1 & 1 \\ - 1 0 & - 0. 5 \end{array} \right] ^ {- 1} \left[ \begin{array}{c} 9 5. 8 \\ - 2 0 0 \end{array} \right] = \left[ \begin{array}{c} 1 6. 0 \\ 7 9. 8 \end{array} \right],
$$

and thus $h _ { \mathrm { l b } } ( t ) \ = \ 1 6 . 0 e ^ { - 1 0 t } + 7 9 . 8 e ^ { - 0 . 5 t } , \forall t \ \geq \ 0$ . This lower bound $h _ { \mathrm { l b } } ( t )$ prevents $h ( t )$ from decaying below it. Since $h ( t )$ would drop under $h _ { \mathrm { l b } } ( t )$ if the robot moves too quickly, the controller reduces its speed to avoid this violation. Figure 2c visualizes this lower bound in dark purple, and we see that it closely matches the actual lower bound, which validates Corollary 1. Note that for the other parameter pair, we compute the lower bound with the initial conditions at $t _ { 0 } \approx 0 . 3 \mathrm { s }$ , which is the moment when the CBF condition becomes active and $h ( t )$ reaches the actual lower bound. The lower bound computed at an earlier time would only loosely bound $h ( t )$ and could not convincingly validate Corollary 1. 

# C. Second Experiment: Our TTCBF Approach

In this experiment, we apply our TTCBF approach proposed in Sec. III-B. Similar to the standard discrete-time CBF approach, we require $\lambda _ { 1 } \in ( 0 , 1 ]$ . We run simulations over a dense grid of $\lambda _ { 1 } \in [ 0 . 0 1 , 0 . 5 0 ]$ , which covers a broad range of control behaviors in this experiment. 

Figure 3a shows the mean $x$ -speed for each $\lambda _ { 1 }$ . Straightforwardly, larger values of $\lambda _ { 1 }$ lead to higher mean $x$ -speed, simplifying the design compared to the standard HOCBF approach that requires the joint tuning of multiple class $\kappa$ functions. Figure 3b shows the robot footprints, and Figure 3c selectively shows the CBF values and their corresponding lower bounds for two distinct parameter settings. The lower bounds, shown as dotted curves with circle markers, are computed using (7), which is derived from standard discretetime CBFs. As observed, our approach imposes a lower bound that matches this analytical lower bound. 

# V. DISCUSSIONS

The experiment in Sec. IV-B demonstrates that the performance of the HOCBF approach is jointly influenced by the parameters of the class $\kappa$ functions, and tuning even two parameters can be challenging. This complexity can grow significantly for constraints with higher relative degrees, which require more class $\kappa$ functions. Our TTCBF approach addresses this challenge by requiring only one class $\kappa$ function. While the standard CBF approach remains preferable for constraints with relative degree one, the advantages of our approach become evident for higher relative degrees, where it simplifies the tuning process compared to the standard HOCBF approach. 

Our TTCBF approach uses a truncated Taylor series to approximate the forward difference $\Delta h$ in the discrete-time CBF condition (5), leading to a more constrained condition due to the additional term $\begin{array} { r l } { ~ } & { { } \left( \gamma - \frac { \Gamma _ { r + 1 } } { ( r + 1 ) ! } \right) \Delta t ^ { r + 1 } } \end{array}$ − in (22). This inherently introduces conservatism. However, note that this term is scaled by $\Delta t ^ { r + 1 }$ . When $\Delta t ^ { r + 1 }$ is sufficiently small, 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-15/2a1ca23f-107c-4ddc-83d7-f794bac18556/29b5ed91a9702845ddb6fddc7f3cbb1913ea3ef0657ed211376486e4e5878dce.jpg)



(a) Mean x-speed.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-15/2a1ca23f-107c-4ddc-83d7-f794bac18556/4b3d2ed50782d5a8b4429936e9ab1c05f9bbb5526868d6b0917b12808e425892.jpg)



(b) Footprint.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-15/2a1ca23f-107c-4ddc-83d7-f794bac18556/932e7607d1b4c3736caf46c8a040f749d9e6e0d3d469099414a64ef47c5d21bd.jpg)



(c) CBF value $h$ and its analytical lower bound $h _ { \mathrm { l b } }$ computed by (17).



Fig. 2: Experimental results from the standard HOCBF approach with different parameters for the class $\kappa$ functions.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-15/2a1ca23f-107c-4ddc-83d7-f794bac18556/1b7bac1b52059d36bbff0c6e24dd3ba1165c731ccc7ee01ca2845c889caa5020.jpg)



(a) Mean $_ x$ -speed.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-15/2a1ca23f-107c-4ddc-83d7-f794bac18556/6eaf379240daac12477fa7d1360d65d91be5800dc052aa777ea4585e5f465c82.jpg)



(b) Footprint.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-04-15/2a1ca23f-107c-4ddc-83d7-f794bac18556/619849dfb4ef304f61a281ad955b9c53f7dfb3c529254412de30fe00876c7997.jpg)



(c) CBF value $h$ and its analytical lower bound $h _ { \mathrm { l b } }$ computed by (7).



Fig. 3: Experimental results from our Taylor-based approach with different parameters for the class $\kappa$ function.


either due to small $\Delta t$ or large $r$ , this term becomes negligible. In our experiment in Sec. IV-C, with $\Delta t = 0 . 0 1$ s and $r = 2$ , omitting this term did not affect control performance. A sampling period of 0.01 s was feasible because the QP problem (24) could be solved efficiently, averaging 3 ms per step. If this term is not negligible (for example in computationally intensive applications that require larger sampling periods), one needs to determine the design parameter $\gamma$ . By Definition 7, this parameter should be greater than or equal to the upper bound of the $( r + 1 ) \mathrm { t h }$ time derivative of the candidate CBF, denoted by $\Gamma _ { r + 1 }$ . If this upper bound cannot be directly estimated, one can empirically select a constant $\gamma$ that yields satisfying control performance. 

# VI. CONCLUSIONS

In this work, we have derived the explicit form of the condition imposed by the standard HOCBF approach, expressed as a homogeneous linear differential inequality. We have demonstrated that the solution of the corresponding equality represents the imposed lower bound on the CBF value, showing explicitly how the parameters of class $\kappa$ functions and the initial conditions of the candidate CBF determine this lower bound. We proposed our TTCBF approach. It is enabled by using a truncated Taylor series that approximates the discrete-time CBF condition. It simplifies control design for constraints with high relative degrees by requiring only one class $\kappa$ function compared to the HOCBF approach, which requires multiple class $\kappa$ functions. Numerical experiments in a collision-avoidance scenario confirmed that the derived analytical lower bound closely matches the actual ones. They also validated our approach’s ability to handle constraints with high relative degrees while using only one class $\kappa$ function. 

# REFERENCES



[1] A. D. Ames, J. W. Grizzle, and P. Tabuada, “Control barrier function based quadratic programs with application to adaptive cruise control,” in 53rd IEEE Conference on Decision and Control, 2014, pp. 6271– 6278. 





[2] J. Zeng, B. Zhang, and K. Sreenath, “Safety-critical model predictive control with discrete-time control barrier function,” in 2021 American Control Conference (ACC), 2021, pp. 3882–3889. 





[3] A. Alan, T. G. Molnar, A. D. Ames, and G. Orosz, “Parameterized barrier functions to guarantee safety under uncertainty,” IEEE Control Systems Letters, vol. 7, pp. 2077–2082, 2023. 





[4] J. Xu and B. Alrifaee, “Learning-based control barrier function with provably safe guarantees: Reducing conservatism with heading-aware safety margin,” in European Control Conference (ECC), in Press, 2025. 





[5] S.-C. Hsu, X. Xu, and A. D. Ames, “Control barrier function based quadratic programs with application to bipedal robotic walking,” in 2015 American Control Conference (ACC), 2015, pp. 4542–4548. 





[6] Q. Nguyen and K. Sreenath, “Exponential control barrier functions for enforcing high relative-degree safety-critical constraints,” in 2016 American Control Conference (ACC). Boston, MA, USA: IEEE, 2016, pp. 322–328. 





[7] W. Xiao and C. Belta, “Control barrier functions for systems with high relative degree,” in 2019 IEEE 58th Conference on Decision and Control (CDC), 2019, pp. 474–479. 





[8] W. Xiao, C. Belta, and C. G. Cassandras, “Adaptive control barrier functions,” IEEE Transactions on Automatic Control, vol. 67, no. 5, pp. 2267–2281, 2022. 





[9] Y. Xiong, D.-H. Zhai, M. Tavakoli, and Y. Xia, “Discrete-time control barrier function: High-order case and adaptive case,” IEEE Transactions on Cybernetics, vol. 53, no. 5, pp. 3231–3239, 2023. 





[10] H. Ma, B. Zhang, M. Tomizuka, and K. Sreenath, “Learning differentiable safety-critical control using control barrier functions for generalization to novel environments,” in 2022 European Control Conference (ECC), 2022, pp. 1301–1308. 





[11] T. Kim, R. I. Kee, and D. Panagou, “Learning to refine input constrained control barrier functions via uncertainty-aware online parameter adaptation,” in 2025 IEEE International Conference on Robotics and Automation (ICRA), in Press, 2025. 





[12] A. D. Ames, S. Coogan, M. Egerstedt, G. Notomista, K. Sreenath, and P. Tabuada, “Control barrier functions: Theory and applications,” in 2019 18th European Control Conference (ECC). Naples, Italy: IEEE, 2019, pp. 3420–3431. 





[13] A. Agrawal and K. Sreenath, “Discrete control barrier functions for safety-critical control of discrete systems with application to bipedal robot navigation,” in Robotics: Science and Systems XIII. Robotics: Science and Systems Foundation, 2017. 





[14] M. Ahmadi, A. Singletary, J. W. Burdick, and A. D. Ames, “Safe policy synthesis in multi-agent POMDPs via discrete-time barrier functions,” in 2019 IEEE 58th Conference on Decision and Control (CDC), 2019, pp. 4797–4803. 





[15] J. Xu, P. Hu, and B. Alrifaee, “SigmaRL: A sample-efficient and generalizable multi-agent reinforcement learning framework for motion planning,” in 2024 27st International Conference on Intelligent Transportation Systems (ITSC), in Press, 2024. 

