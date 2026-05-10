# SigmaRL: A Sample-Efficient and Generalizable Multi-Agent Reinforcement Learning Framework for Motion Planning

Jianye Xu1 , Student Member, IEEE, Pan Hu2 , Bassam Alrifaee3 , Senior Member, IEEE 

Abstract— This paper introduces an open-source, decentralized framework named SigmaRL, designed to enhance both sample efficiency and generalization of multi-agent Reinforcement Learning (RL) for motion planning of connected and automated vehicles. Most RL agents exhibit a limited capacity to generalize, often focusing narrowly on specific scenarios, and are usually evaluated in similar or even the same scenarios seen during training. Various methods have been proposed to address these challenges, including experience replay and regularization. However, how observation design in RL affects sample efficiency and generalization remains an under-explored area. We address this gap by proposing five strategies to design information-dense observations, focusing on general features that are applicable to most traffic scenarios. We train our RL agents using these strategies on an intersection and evaluate their generalization through numerical experiments across completely unseen traffic scenarios, including a new intersection, an on-ramp, and a roundabout. Incorporating these information-dense observations reduces training times to under one hour on a single CPU, and the evaluation results reveal that our RL agents can effectively zero-shot generalize. 

Code: github.com/bassamlab/SigmaRL 

# I. INTRODUCTION

# A. Motivation

Reinforcement Learning (RL) has become an increasingly promising approach for motion planning of Connected and Automated Vehicles (CAVs), owing to its ability to learn through interactions with the environment. Despite its great success, the generalization of RL agents—i.e., the ability to generalize to unseen scenarios or environments—remains one of the fundamental challenges. 

Most RL agents for CAVs are specialized for a specific scenario, such as an intersection, an on-ramp, or lanefollowing, see [1] for a comprehensive survey. Besides, due to a lack of generalization, they are often tested in a similar or even the same scenario seen during training. This leads to an RL agent being only used in one scenario. It may even fail to generalize to new scenarios or tasks that seem similar to the training scenarios or tasks [2]. 

To enhance their generalization, one strategy involves enriching the diversity of training scenarios, e.g., training 

This research was supported by the Bundesministerium fur Digitales und ¨ Verkehr (German Federal Ministry for Digital and Transport) within the project “Harmonizing Mobility” (grant number 19FS2035A). 

1The author is with the Chair of Embedded Software (Informatik 11), RWTH Aachen University, Germany, xu@embedded.rwth-aachen.de. 

2The author is with the Department of Computer Science, RWTH Aachen University, Germany, pan.hu@rwth-aachen.de. 

3The author is with the Department of Aerospace Engineering, University of the Bundeswehr Munich, Germany, bassam.alrifaee@unibw.de. 

in an environment where different scenarios are involved. However, this strategy tends to impair sample efficiency, as most traffic situations are not challenging, owing to the rarity of safety-critical events [3]. We call this inefficiency sample inefficient. A sample (also called an experience or a frame) in RL refers to a single interaction instance between an agent and an environment. This interaction includes the agent observing the state of the environment, taking an action based on its policy, and receiving feedback in the form of a reward and a new state from the environment. The learning agent uses these samples to update its policy. By inefficient, we mean that the learning agent requires an excessive amount of samples before the policy achieves a satisfactory performance. Formally, we define sample efficiency as follows. 

Definition 1 (Sample Efficiency). Consider a set of RL models M, each trained with a fixed number of samples. For each model $M _ { i } \in { \mathcal { M } } .$ we use the performance metrics: 

• Collision Rate $( C R _ { M _ { i } } ) \mathrm { ~ ~ }$ : The proportion of time steps where agents cause a collision, 

• Center Line Deviation $( C D _ { M _ { i } } ) \mathrm { : }$ The average deviation of all agents from their lane center lines, and 

• Average Speed $( A S _ { M _ { i } } )$ : The average speed of all agents to quantify its sample efficiency by a composite score 

$$
C S _ {M _ {i}} := - w _ {1} \cdot C R _ {M _ {i}} - w _ {2} \cdot C D _ {M _ {i}} + w _ {3} \cdot A S _ {M _ {i}}, \tag {1}
$$

where w1, w2, and $w _ { 3 }$ are weighting factors to balance the relative importance and scale of the performance metrics. The model $M ^ { * } \ = \ \arg \operatorname* { m a x } _ { M _ { i } \in \mathcal { M } } C S _ { M _ { i } }$ that maximizes the composite score $C S _ { M _ { 1 } }$ among all models $M _ { i } \in \mathcal { M }$ is deemed the most sample-efficient. 

The above-mentioned sample inefficiency is further amplified by the reliance on raw-sensor-data-based end-to-end learning paradigms, requiring RL agents to learn a direct mapping from raw sensor data to actions. Direct learning from raw sensor data may allow agents to uncover patterns and correlations that might be missed with handcrafted features. However, it demands sophisticated feature extraction capabilities, typically provided by deep Convolutional Neural Networks (CNNs), which significantly increases the complexity of the learning process. 

In light of these challenges, we identify a need to develop an effective approach toward sample-efficient and generalizable RL for CAVs. 

# B. Related Work

Despite the success of RL, the training process often demands an extensive amount of samples in real-world applications [4], ranging from tens to hundreds of millions, casting sample efficiency important ongoing research. Additionally, generalization is vital for RL agents, especially in real-world applications where environments are unpredictable and it is impractical to generate training data that can cover all possible situations. Consequently, generalization in RL has gained substantial attention in recent years. Below, we briefly overview recent works that address sample efficiency and generalization in RL. 

Sample Efficiency: In [5], the sample efficiency of RL from multiple aspects is discussed, such as more effective environmental exploration and better policy optimization. Another concern in RL is the sequential dependency of experiences—an alternative term for samples, which violates the independent and identically distributed (i.i.d.) assumption that underlies many stochastic gradient descent algorithms for RL. Experience replay in [6] addresses this concern by storing experiences in a memory buffer and randomly sampling from it when learning, instead of learning directly from incoming experiences. This method largely enhances sample efficiency, making it a new standard in many RL algorithms [7]. To make learning more efficient, [8] proposes a strategy called Prioritized Experience Replay, which assigns greater importance to certain experiences based on a prioritization scheme, ensuring that more crucial experiences are sampled with higher frequency. Two recent works [9], [10] apply RL for autonomous drone racing, showcasing champion-level racing performance in their experiments. It uses an initial state buffer to store successful states for agents navigating through a gate, and it has been highlighted that sampling from this buffer during environment resets significantly enhances sample efficiency. Another work [11] proposes an approach for integrating model-free and modelbased RL to combine the high performance of the former with the reduced sample complexity of the latter. Further, [12] examines the sufficiency of a good representation of function approximation for achieving sample-efficient RL, particularly regarding value functions, transition models, reward functions, and policies. Despite these advances, a gap remains in research specifically targeting the design of observations to boost the sample efficiency of RL agents. 

Generalization: The authors in [2] explore the potential of regularization techniques such as dropout to enhance both the sample efficiency and generalization of Deep Q-Learning and show that it can foster the learning of more general features. Meanwhile, [13] presents an empirical comparison of two prominent RL algorithms, A2C [14] and Proximal Policy Optimization (PPO) [15], examining their generalization in conjunction with EPOpt [16] and RL2 [4], two methods aimed at addressing the generalization problem of RL. Further, [17] introduces an RL environment called Coin-Run, a benchmark for assessing the generalization of RL. This benchmark provides metrics to quantify how various factors—like neural network structures and regularization techniques—impact the generalization. In [18], a simulation platform called MetaDrive is developed to facilitate the research of generalizable RL agents for autonomous driving, accommodating both single- and multi-agent settings. In addition, the conducted experiments therein show that diversifying training scenarios improves the generalization of RL agents. Observing the non-stationarity in on-policy RL, [19] proposes a hypothesis that neural networks exhibit a memory effect, which can harm the generalization when training data distributions shift. It proposes an approach called Iterate Relearning to counteract this effect. Zero-shot generalization represents a specific type of generalization, referring to an RL agent’s ability to generalize to completely unseen scenarios without further training or fine-tuning. A recent survey [20] categorizes approaches to realize zeroshot generalization, highlighting strategies to either increase the similarity, decrease the difference between training and testing data, or enhance optimization to prevent overfitting. Nevertheless, the majority of the state-of-the-art research focuses on improving the generalization of RL agents on the algorithm level. 

In summary, while substantial research efforts have contributed to enhancing RL agents’ sample efficiency and generalization, a notable research gap remains in exploring how observation design could further improve them. Observation design in RL, particularly for CAVs, is crucial because it determines the quality of environmental information that the learning agents receive. Poorly designed observations can lead to inefficient and ineffective learning, as the agents might incorrectly interpret their surroundings. 

# C. Paper Contributions

The main contributions of this paper are twofold: 

• It presents an open-source, decentralized Multi-Agent Reinforcement Learning (MARL) framework named SigmaRL for motion planning of CAVs. The RL agents within this framework require less than one hour of training time on a single CPU and can zero-shot generalize to completely unseen traffic scenarios. 

• It examines how observation design influences sample efficiency and generalization of RL agents—an underexplored area within the RL community. As outcomes, it proposes five strategies for designing information-dense, structured observations for motion planning, opposite to bulky, image-based observations. They are 1) using an ego view instead of a bird-eye view, 2) observing vertices of surrounding agents instead of their poses and geometric dimensions, 3) observing distances to surrounding agents, 4) observing distances to lane boundaries instead of points sampled from them, and 5) observing distances to lane center lines. 

# D. Notation

A variable x is marked with a superscript x(i) if it belongs to agent i. All other information is presented in its subscript. For example, the value of x at time t is written as xt. If multiple pieces of information need to be conveyed in subscript, they are separated by commas. For any set S, the cardinality of the set is denoted by |S|. We use the term agent to refer interchangeably to a vehicle or an RL agent. 

# E. Paper Structure

Section II formally formulates the problem. Section III presents our framework as a solution, including the RL algorithm, the environment, the observation design, and the reward design. Section IV details experiment setups and discusses the experiment results. Section V draws conclusions and outlines future research. 

# II. PROBLEM FORMULATION

We consider the problem of MARL for motion planning of CAVs in a discrete-time setting, where a set of agents interact in a shared environment, aiming to achieve both safety and efficiency in traffic flow. This problem can be described using a Markov Game, also known as a stochastic game [21], which is a standard multi-agent setting of a Markov Decision Process. In addition, we consider agents have limited sensor capabilities, and they can only sense their surrounding environments, making the problem more realistic. This condition is known as partial observability, leading to the so-called Partially Observable Markov Game (POMG), which is challenging to solve due to imperfect information of the game [22]. Formally, we define a POMG as follows. 

Definition 2 (Adapted from [23]). A POMG is defined by a tuple $( \mathcal { N } , \mathcal { S } , \{ \mathcal { A } ^ { ( i ) } \} _ { i \in \mathcal { N } } , \{ \mathcal { O } ^ { ( i ) } \} _ { i \in \mathcal { N } } , \mathcal { P } , \{ R ^ { ( i ) } \} _ { i \in \mathcal { N } } , \gamma )$ , where 

• $\mathcal { N } = \{ 1 , \ldots , N \}$ denotes a finite set of agents. 

• $s$ is the state space of the system shared by all agents. 

• $\mathcal { A } ^ { ( i ) }$ denotes the action space of agent i. The jointaction space of all agents is the Cartesian product:$\mathcal { A } : = \times _ { i \in \mathcal { N } } \mathcal { A } ^ { ( i ) }$ . The joint action of all agents at timestep t is denoted as $\mathbf { \delta } _ { \mathbf { { a } } _ { t } } : = ( a _ { t } ^ { ( 1 ) } , \dots , a _ { t } ^ { ( N ) } )$ (N) , where $a _ { t } ^ { ( i ) }$ （24号denotes the action of agent i at time step t.

• $\mathcal { O } ^ { ( i ) }$ is the observation space of agent i. Similarly, $\mathcal { O } : =$ $\times _ { i \in \mathcal { N } } \mathcal { O } ^ { ( i ) }$ and $\mathbf { } _ { o _ { t } }$ denote the joint observation space and the joint observations of all agents at time step t, respectively. 

• $\mathcal { P } : \mathcal { S } \times \mathcal { A }  \Delta ( \mathcal { S } ) \times \Delta ( \mathcal { O } )$ is the Markovian state transition and observation probability function, which describes the probability of transitioning from one state to another and agents getting certain observations given the current state and the joint actions of all agents. $\Delta$ denotes a probability distribution over a space. 

• $R ^ { ( i ) } : \mathcal { S } \times \mathcal { A } \times \mathcal { S } \to \mathbb { R }$ is the reward function that determines the immediate reward received by agent i for a transition from $\left( { { s _ { t } } , { a _ { t } } } \right)$ to $s _ { t + 1 }$ , where $s _ { t }$ and $s _ { t + 1 }$ are the environment states before and after the transition, and $\mathbf { } \mathbf { a } _ { t }$ is the joint action. We denote the joint rewards at time step t with $\mathbf { } _ { \mathbf { } } ^ { \mathbf { } } \mathbf { \Delta } \mathbf { r } _ { t }$ . 

• $\gamma ~ \in ~ [ 0 , 1 )$ is the discount factor that balances the immediate and future rewards. 

At any given time step t, each agent i undertakes an action a(i) $a _ { t } ^ { ( i ) }$ in response to its partial observation $o _ { t } ^ { ( i ) }$ . Following this, the environment transitions to a new stateach agent i through the reward function $s _ { t + 1 }$ $R ^ { ( i ) } ( s _ { t } , a _ { t } ^ { ( i ) } , s _ { t + 1 } )$ Problem 1. Considering a POMG defined in Definition 2, design partial observations that allow each agent i to efficiently learn a policy $\pi ^ { ( i ) } : O _ { i } \to \Delta ( A ^ { ( i ) } )$ ), which is a mapping from its partial observation $o ^ { ( i ) }$ to a probability distribution over its action space $\mathcal { A } ^ { ( i ) }$ . The design of partial observations should maximize the sample efficiency defined in Definition 1. The agents should be applicable to motion planning of CAVs and can be zero-shot generalized to unseen traffic scenarios. 

Note that the single-agent setting of a POMG is already Nondeterministic Exponential Time Complete (NEXP-Complete), requiring super-exponential time to find the optimal solution in the worst case [24]. Instead of trying to find the optimal solution, we focus on how to design observations to improve the sample efficiency and generalization of RL agents. 

# III. OUR FRAMEWORK

In this section, we detail our framework SigmaRL. We describe its RL algorithm in Sec. III-A and its RL environment in Sec. III-B. We propose the observation-design strategies in Sec. III-C. Reward design is out of our scope in this work, and we refer to our open-source repository for details. 

# A. RL Algorithm

We employ a multi-agent extension of PPO, termed multiagent PPO [25], as our RL algorithm. PPO is a widely adopted RL algorithm that uses gradients to adjust policies toward more rewarding actions, featuring an architecture that includes both an actor to make decisions and a critic to evaluate these decisions. Multi-agent PPO extends it to a multiagent setting, using a centralized critic alongside distributed actors, a popular learning scheme known as centralizedlearning-decentralized-execution. Note that the centralized critic is only needed during training. This scheme is particularly designed to mitigate the non-stationary problem in MARL, especially under conditions of partial observability [26]. Besides, we consider homogeneous agents, enabling shared actor parameters and more efficient learning through experience sharing. 

Figure 1 overviews our decentralized framework aiming to solve Problem 1. At time step t, each agent i receives its partial observation $o _ { t } ^ { ( i ) }$ from its designated observer, Observer(i), and executes its policy via its actor, Actor(i), to generate action $a _ { t } ^ { ( i ) }$ . Subsequently, the environment updates its state based on the joint actions $\mathbf { } \mathbf { a } _ { t }$ of all agents. The critic, serving as a state-value function, estimates the potential future reward $V ( o _ { t } )$ based on the joint observations of all agents $\mathbf { } o _ { t } .$ . The loss module updates the critic and actor, details of which are referred to [15], [25]. 

# B. RL Environment

We use VMAS [27], a vectorized multi-agent simulator for collective robot learning, as our RL environment framework. We customize the environment to align with our Cyber-Physical Mobility Lab [28], an open-source testbed for CAVs. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-05-10/ccb73a48-3432-4fdf-bb0f-edd249b152fd/436521f5e1cc2b8aba5835553a77943dfa11d572dc66f8c327921112b67e9406.jpg)



Fig. 1: Overview of the proposed decentralized MARL framework SigmaRL. t: time step; $i \in \{ 1 , \ldots , N \}$ : agent index.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-05-10/ccb73a48-3432-4fdf-bb0f-edd249b152fd/558e40c805a8f84f28561de530b2f665f229488047e60c7ccf2d70b0b78449fb.jpg)



Fig. 2: Kinematic single-track model. C: Center of Gravity; x, y: x- and y-coordinates; v: velocity; β: slide slip angle; ψ: yaw angle; δ: steering angle; L: wheelbase.


We use the nonlinear kinematic single-track model [29, Sec. 2.2], visualized in Fig. 2, to model the agent dynamics. It uses speed v and steering angle δ as control actions. We use continuous action spaces with $v \in [ - 0 . 8 , 0 . 8 ] ~ \mathrm { m / s }$ and $\delta \in [ - 3 5 , 3 5 ] ^ { \circ }$ . We consider each agent a rectangle with a width of 0.08 m and a length of 0.16 m. 

# C. Observation Design

In this section, we propose five observation-design strategies to enhance the sample efficiency and the generalization of RL agents: the first one concerns the coordinate system used to represent surroundings, the second and third pertain to surrounding agents, and the last two are related to lanes. We verify the effectiveness of these strategies in the ablation studies in Sec. IV. 

Most state-of-the-art RL agents for motion planning use image-like observations, which hold the information in an unstructured manner, usually requiring deep neural network architectures such as deep CNNs to extract relevant features. However, using image-like observations hardens the learning process and often leads to large samples and time to converge [30], which contradicts the objective of this work. Therefore, we use structured data with dense information to represent relevant information for motion planning, allowing the use of shallower neural networks for learning. 

1) Use Ego View: Mainly two types of presentations of surroundings are used in the motion planning of CAVs: Ego view and bird-eye view (also known as top-down view). While the bird-view presentation of surroundings is popular in traditional optimization-based methods, machine learning-based methods favor both. Bird-view presentation is 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-05-10/ccb73a48-3432-4fdf-bb0f-edd249b152fd/cc41d6519c8417c0e4bfe28dc543317a9a72c76a6c01518ebd00771736f7d688.jpg)



Fig. 3: Observations of agent i. Red: efficient observation (ours). Green: inefficient observation (not ours).


particularly advantageous in imitation learning for generating training data and for representing spatial relationships in a human-readable format [31]. However, we suggest using ego-view representation in RL, as it is naturally used in biological systems and thus adheres to the fact that most core algorithms of RL were inspired by biological learning systems [32, p. 4]. The upcoming ablation studies in Sec. IV confirm its significant ability in enhancing sample efficiency and generalization. 

2) Vertices of Surrounding Agents: Agents need to observe their surrounding agents to make risk-aware motion planning. One traditional way is to observe their poses1 along with their geometric dimensions like lengths and widths, with the hope that agents implicitly learn the occupancy2 of their surrounding agents. However, we suggest explicitly observing that occupancy. We propose to represent an agent’s occupancy with the coordinates of its vertices. See Fig. 3 as an example, where the ego agent i observes the vertices shown in red points of its surrounding agent j. Our ablation studies in Sec. IV proves that this would notably lower the collision rate between agents, which we term agent-agent collision rate. 

3) Distances to Surrounding Agents: Beyond observing the occupancy of surrounding agents, we suggest additionally observing the distances to them. Figure 3 depicts the ego agent i’s distance to its surrounding agent $j ,$ denoted as $d _ { j } ^ { ( \bar { i } ) }$ . 

Our ablation studies in Sec. IV show that this strategy would further reduce the agent-agent collision rate. 

4) Distances to Lane Boundaries: Essentially, agents need to observe lane boundaries to prevent collisions with them (also called off-road events). One traditional approach to represent a lane boundary is to discretize it with a polyline, such as a Lanelet [33], [34], and observe the coordinates of the points sampled from the polyline. See Fig. 3 as an example, where the sampled points of the lane boundaries of the ego agent i are depicted by green points. However, we remark that this approach, despite being widely used, is inefficient, since the number of sampled points may be large to preserve fidelity. To counteract this shortcoming, we propose a more compact observation approach—observing the distances to lane boundaries. As an example, see ai’s distances to its left and right boundaries, denoted as $\bar { d } _ { \mathrm { L B } } ^ { ( i ) }$ and $d _ { \mathrm { R B } } ^ { ( i ) }$ LB, respectively. Our ablation studies in Sec. IV reveal that this approach would remarkably reduce the collision rate between agents and lane boundaries, which we term agentlane collision rate. 

5) Distances to Lane Center Lines: The observation of lane center lines contributes to lane-following performance. In this work, for each agent at each time step, we dynamically sample a fixed number of the most front points on its lane center line, serving as its short-term reference path. See the three red points in front of agent i in Fig. 3 as an example. We propose to let each agent observe its deviation from its lane center line, depicted exemplarily by $d _ { \mathrm { C L } } ^ { ( i ) }$ in Fig. 3, which denotes agent i’s distance to its lane center line. Our ablation studies in Sec. IV demonstrates that this would greatly increase agents’ lane-following performance. 

Except for the above five observation-design strategies, we let each agent observe its own speed and the relative velocities of its surrounding agents. Adhering to partial observability, we allow each agent to observe only a limited number of surrounding agents. 

On the left side, Fig. 1 overviews each agent i’s observation. At each time step t, agent i’s observation consists of two parts: self-observation $\bar { o } _ { t , \mathrm { s e l f } } ^ { ( i ) }$ and the observation of surrounding agents o t,sur.. $o _ { t , \mathrm { s u r . } } ^ { ( i ) }$ Specifically, the self-observation $o _ { t , \mathrm { s e l f } } ^ { ( i ) }$ consists of its own speed, a short-term reference path sampled from its lane center line, distance to this center line, and distances to its left and right lane boundaries. This results in a total of $4 + 2 \cdot n _ { \mathrm { p , R P } }$ data points, where $n _ { \mathrm { p , R P } }$ denotes the number of points building the short-term reference path. Let N (i)t,sur. $\mathcal { N } _ { t , \mathrm { s u r } } ^ { ( i ) }$ denote the set of surrounding agents observable by agent i. The observation of each agent $j \in \mathcal { N } _ { t , \mathrm { s u r . } } ^ { ( i ) } , o _ { t ^ { ( j ) } } ^ { ( i ) }$ , consists of agent $j ^ { \dagger } \mathfrak { s }$ s vertices, velocity, and the distance to agent i, totaling to eleven data points. Consequently, this yields the observation of surrounding agents at time step t: $\begin{array} { r } { \dot { o } _ { t , \mathrm { s u r . } } ^ { ( i ) } : = \bigcup _ { j \in \mathcal { N } _ { t , \mathrm { s u r . } } ^ { ( i ) } } o _ { t ^ { ( j ) } } ^ { ( i ) } } \end{array}$ o t,sur. Sj∈N (i)t,sur. . Agent i stacks the above two parts to Vt,sur. form its final observation at time step t: ot $o _ { t } ^ { ( i ) } : = o _ { t , \mathrm { s e l f } } ^ { ( i ) } \cup o _ { t , \mathrm { s u r . } } ^ { ( i ) }$ . Denoting further by $n _ { \mathrm { s u r . } } ^ { ( i ) } : = \left| \mathcal { N } _ { t } ^ { ( i ) } \right|$ the number of observed surrounding agents yields the total observation size of each agent i: o(i) $\left| o _ { t } ^ { \left( i \right) } \right| ^ { - } = 4 + 2 \cdot n _ { \mathrm { p , R P } } + 1 1 \cdot n _ { \mathrm { s u r . } } ^ { \left( i \right) }$ . 

Remark 1 (Generalization and Sample Efficiency). The proposed dense representation of observations crafts general yet crucial features for motion planning, such as vertices of surrounding agents and distances to them, as well as distances to lane boundaries and center lines. These features are broadly applicable across nearly all traffic scenarios, which grants RL agents a strong generalization potential to handle unseen scenarios. This way, we are allowed to train them on a challenging scenario to effectively learn a generalizable policy, without overfitting to specific scenes. Moreover, unlike end-to-end motion planning methods that demand sophisticated feature extractors, this approach densifies the information in observations to facilitate the learning process, thereby enhancing sample efficiency. 

Remark 2 (Practicability). The proposed observation-design strategies necessitate observing the distances to lane boundaries, lane center lines, and distances to lane center lines. This poses no issue if a high-definition map representing lane boundaries and center lines exists and if agents can localize themselves within it, making it possible to calculate this information. Besides, the proposed strategies also necessitate observing the vertices of surrounding agents and the distances to them, which can be realized through communication. Given that agents can localize themselves within the map, they know their poses and can calculate the coordinates of their vertices based on their geometric dimensions. They can then communicate these coordinates to other agents that need this information. 

# IV. EXPERIMENTS

In this section, we conduct five ablation studies to validate the efficacy of the observation-design strategies proposed in Sec. III-C and use four unseen scenarios to test our RL agents’ generalization. We present the experiment setups in Sec. IV-A and discuss the experiment results in Sec. IV. We open-source our repository3 for reproducibility and provide a video therein to demonstrate our experiments. 

# A. Training and Testing Setups

In our multi-agent PPO, the centralized critic has four layers, one input layer, two hidden layers, and one output layer, with each hidden layer having 256 nodes. We use Tanh as the activation function. The actor has the same neural network architecture as the critic. We use a sample time of 50 ms for both training and testing. We set the number of observed surrounding agents $n _ { \mathrm { s u r } }$ . to two and the number of points on a short-term reference path $n _ { \mathrm { p , R P } }$ sampled from a lane center line to three. We use a discount factor γ of 0.99. 

We train six RL models, i.e., $\mathcal { M } = \{ M _ { 0 } , \ldots , M _ { 5 } \} . \ M _ { 0 }$ , called our model thereafter, incorporates all five observationdesign strategies proposed in Sec. III-C. From models $M _ { 1 }$ to $M _ { 5 }$ , each model omits one of these strategies in order. For instance, instead of using an ego view, $M _ { 1 }$ uses a birdeye view. Since our proposed observation-design strategies focus on general features that are applicable to most traffic scenarios, we are allowed to train the models only on the intersection of the CPM Scenario to learn generalizable policies. Besides, we train each model with only four agents but test them with more agents. We set the number of training episodes to 250, where 4096 samples are collected per episode, leading to approximately only one million samples. We predefine several long-term reference paths and randomly select one for each agent. Besides, we initialize the agents with random initial states. Once a collision occurs, we reset all agents with random states and also with randomly selected reference paths from the predefined reference paths. To ensure the feasibility of the initial states, we ensure the initial distances between agents are large enough (larger than 1.2 times the diagonal lengths of the agents). 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-05-10/ccb73a48-3432-4fdf-bb0f-edd249b152fd/5f81124025203ea90182537a6b349d18eb6b264686ce7ec96128527fe381fbc8.jpg)



(a) CPM Scenario.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-05-10/ccb73a48-3432-4fdf-bb0f-edd249b152fd/39c9eb405aab409cdbdcff1361d9baa32027de295ff9359f1a901e7f799bda36.jpg)



(c) On-Ramp Scenario.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-05-10/ccb73a48-3432-4fdf-bb0f-edd249b152fd/f958ec182fd558aeae0c7482b9d377ea9c098b5121ff0f8acaecea8d2a6c6311.jpg)



(b) Intersection Scenario.


![image](https://cdn-mineru.openxlab.org.cn/result/2026-05-10/ccb73a48-3432-4fdf-bb0f-edd249b152fd/17a3403d96b4cfd0d43fdf1ae30fd740aaad21095397d1f670da6b84ff12671e.jpg)



(d) Roundabout Scenario.



Fig. 4: Training and testing scenarios. Train only on the intersection of the CPM Scenario (see gray area). Test in all four scenarios, with the depicted numbers of agents.


We test the models in four completely unseen scenarios to evaluate the generalization of the learned policies: the entire CPM Scenario (Fig. 4a) and three other scenarios handcrafted in Open Street Map [35]: an intersection (Fig. 4b), an on-ramp (Fig. 4c), and a roundabout (Fig. 4d), with 15, 6, 8, and 8 agents, respectively. Since we train agents only on the intersection of the CPM Scenario, all four scenarios are unseen4 for them, which significantly challenges their generalization ability. To obtain convincing results, we conduct 32 simulations in each scenario for each model, with each simulation having 1200 time steps. Since the sample time is 50 ms, each simulation lasts for 50 ms × 1200 = 1 min. In each simulation, we initialize the agents with random initial states. Once a collision occurs, we reset only the colliding agents so that other agents will not be unnecessarily reset to “safe” states. 

For each simulation and for each model, we compute three performance metrics: collision rate CR, center line deviation CD, and average speed AS, which are then used to compute the composite score CS via Eq. (1) to assess the model’s sample efficiency defined in Definition 1. Recall that we distinguish between agent-agent collision rate and agent-lane collision rate, denoted as $C R _ { \mathrm { { A - A } } }$ and $C R _ { \mathrm { A - I } }$ , respectively. Summing them up yields the total collision rate $C R _ { \mathrm { t o t a l } } : = C R _ { \mathrm { A - A } } + C R _ { \mathrm { A - L } }$ . We average the performance metrics over all simulations and show them in Table I. Computing the composite score CS defined by Eq. (1) necessitates determining the weighting factors $w _ { 1 } , w _ { 2 } .$ , and w3. The weighting factors are expected to balance the relative importance and scale of each evaluation metric. We treat each metric equally important and determine the weighting factors by inverting the average of the three performance metrics over all models, i.e., $w _ { 1 } = | \mathcal { M } | / \sum _ { \forall M _ { i } \in \mathcal { M } } C R _ { \mathrm { t o t a l } , M _ { j } } , w _ { 2 } =$ $\boldsymbol { | \mathcal { M } | } / \sum _ { \forall \ : M _ { i } \in \mathcal { M } } \boldsymbol { C } \boldsymbol { D } _ { M _ { j } }$ , and $w _ { 3 } = | \mathcal { \bar { M } } | / \textstyle \sum _ { \forall M _ { i } \in \mathcal { M } } A S _ { M _ { j } }$ , where $| { \mathcal { M } } |$ denotes the number of models, which is six in our case. This way, we balance the scale of the three performance metrics. In summary, the composite $\mathrm { s c o r e } ^ { 5 } ~ C S _ { M _ { i } }$ of model $M _ { i \in \{ 0 , \dots , 5 \} }$ , is calculated as 

$$
C S _ {M _ {i}} = - \frac {| \mathcal {M} | \cdot C R _ {M _ {i}}}{\sum_ {\forall M _ {j} \in \mathcal {M}} C R _ {\text {total,} M _ {j}}} - \frac {| \mathcal {M} | \cdot C D _ {M _ {i}}}{\sum_ {\forall M _ {j} \in \mathcal {M}} C D _ {M _ {j}}} + \frac {| \mathcal {M} | \cdot A S _ {M _ {i}}}{\sum_ {\forall M _ {j} \in \mathcal {M}} A S _ {M _ {j}}}. \tag {2}
$$

# B. Results and Discussions

Figure 5 depicts the mean reward per episode of the six models during training. Due to the dense representation of observations, training each model takes less than one hour on a single CPU (Apple M2 pro with 16 GB of RAM, less than 15 % CPU utilization). Notably, our model $M _ { 0 }$ , which incorporates all five observation-design strategies, exhibits the fastest learning speed and the highest episode reward. In contrast, model $M _ { 1 }$ , utilizing a bird-eye view rather than an ego view, demonstrates the lowest learning efficiency. Moreover, model $M _ { 3 }$ , which omits the third observationdesign strategy—observing distances to surrounding agents, also shows lower learning efficiency. Models $M _ { 2 } , M _ { 3 }$ , and $M _ { 5 }$ have similar learning curves, suggesting that the second, third, and fifth observation-design strategies contribute similarly to the learning process. 

Table I lists the testing results for all six models in four scenarios depicted in Fig. 4. We highlight the best performance metric in bold and the best composite score with an ellipse for each row. Model $M _ { 1 }$ , employing a birdeye view, acts overly conservatively in the Intersection, On-Ramp, and Roundabout Scenarios, as evidenced by its nearly zero average speed AS. This conservativeness understandably yields a collision rate close to zero. Therefore, we exclude their performance metrics in these three scenarios from the candidates of the best values. 

Remarkably, despite being trained exclusively on the intersection of the CPM Scenario, our model $M _ { 0 }$ demonstrates robust performance in the unseen scenarios. This achievement confirms that our proposed observation-design strategies successfully grant the RL agents the capability to zero-shot generalize to unseen scenarios. The agents achieve a collision rate of less than 1.0 % while maintaining high traffic efficiency in the testing scenarios. We gauge traffic efficiency using the performance metric average speed AS. Given the maximum speed being set to $0 . 8 \mathrm { m } / \mathrm { s } .$ , our agents achieve average speeds of more than 80 % of this maximum in the testing scenarios. Note that traffic density— specifically, the number of agents—significantly influences traffic efficiency. For intuition, we refer readers to the video within our open-source repository. 

![image](https://cdn-mineru.openxlab.org.cn/result/2026-05-10/ccb73a48-3432-4fdf-bb0f-edd249b152fd/3ea31e91e79678ced8af54cdaa04725ae72581daa822c5aaa6cd9153c095d76f.jpg)



Fig. 5: Mean reward per episode when training the six models $M _ { i \in \{ 0 , \dots , 5 \} }$ . Model $M _ { 0 }$ incorporates all five observationdesign strategies we proposed in Sec. III-C, whereas models $M _ { 1 }$ to $M _ { 5 }$ each omit one of these strategies.



TABLE I: Testing results of the six models in four scenarios. $C R _ { \mathrm { { A ^ { - } A } } } { : }$ agent-agent collision rate; $C R _ { \mathrm { { A - L } } } { : }$ agentlane collision rate; $C R _ { \mathrm { t o t a l } } { : }$ total collision rate; C D: center line deviation; AS: average speed; CS: composite score calculated by Eq. (2).


<table><tr><td colspan="2"></td><td><eq>M_0</eq></td><td><eq>M_1</eq></td><td><eq>M_2</eq></td><td><eq>M_3</eq></td><td><eq>M_4</eq></td><td><eq>M_5</eq></td></tr><tr><td rowspan="6">CPM Scenario</td><td><eq>CR_{A-A}</eq>[%]</td><td>0.04</td><td>4.14</td><td>0.56</td><td>0.92</td><td>0.05</td><td>0.62</td></tr><tr><td><eq>CR_{A-L}</eq>[%]</td><td>0.35</td><td>21.83</td><td>0.02</td><td>0.01</td><td>0.52</td><td>0.01</td></tr><tr><td><eq>CR_{total}</eq>[%]</td><td>0.38</td><td>25.97</td><td>0.58</td><td>0.93</td><td>0.57</td><td>0.63</td></tr><tr><td>CD [cm]</td><td>5.18</td><td>16.03</td><td>4.50</td><td>4.28</td><td>4.60</td><td>5.06</td></tr><tr><td>AS [m/s]</td><td>0.74</td><td>0.43</td><td>0.69</td><td>0.72</td><td>0.73</td><td>0.72</td></tr><tr><td>CS</td><td>0.24</td><td>-7.15</td><td>0.23</td><td>0.23</td><td>0.28</td><td>0.17</td></tr><tr><td rowspan="6">Intersection</td><td><eq>CR_{A-A}</eq>[%]</td><td>0.10</td><td>0.02</td><td>1.33</td><td>2.42</td><td>0.88</td><td>1.76</td></tr><tr><td><eq>CR_{A-L}</eq>[%]</td><td>0.86</td><td>0.20</td><td>0.03</td><td>1.73</td><td>0.47</td><td>1.25</td></tr><tr><td><eq>CR_{total}</eq>[%]</td><td>0.96</td><td>0.22</td><td>1.35</td><td>4.16</td><td>1.35</td><td>3.01</td></tr><tr><td>CD [cm]</td><td>2.76</td><td>2.44</td><td>2.60</td><td>3.64</td><td>2.47</td><td>3.59</td></tr><tr><td>AS [m/s]</td><td>0.71</td><td>0.07</td><td>0.70</td><td>0.72</td><td>0.70</td><td>0.74</td></tr><tr><td>CS</td><td>-0.30</td><td>-0.85</td><td>-0.47</td><td>-2.32</td><td>-0.42</td><td>-1.64</td></tr><tr><td rowspan="6">On-Ramp</td><td><eq>CR_{A-A}</eq>[%]</td><td>0.09</td><td>0.01</td><td>0.55</td><td>3.56</td><td>0.49</td><td>2.56</td></tr><tr><td><eq>CR_{A-L}</eq>[%]</td><td>0.00</td><td>0.01</td><td>0.00</td><td>0.00</td><td>0.00</td><td>0.00</td></tr><tr><td><eq>CR_{total}</eq>[%]</td><td>0.09</td><td>0.03</td><td>0.55</td><td>3.56</td><td>0.49</td><td>2.56</td></tr><tr><td>CD [cm]</td><td>2.00</td><td>2.38</td><td>2.16</td><td>4.15</td><td>2.44</td><td>3.74</td></tr><tr><td>AS [m/s]</td><td>0.69</td><td>0.06</td><td>0.68</td><td>0.71</td><td>0.68</td><td>0.74</td></tr><tr><td>CS</td><td>0.38</td><td>-0.77</td><td>-0.08</td><td>-3.21</td><td>-0.13</td><td>-2.19</td></tr><tr><td rowspan="6">Roundabout</td><td><eq>CR_{A-A}</eq>[%]</td><td>0.26</td><td>0.09</td><td>2.10</td><td>4.78</td><td>1.20</td><td>3.59</td></tr><tr><td><eq>CR_{A-L}</eq>[%]</td><td>0.07</td><td>1.21</td><td>0.00</td><td>0.37</td><td>0.06</td><td>0.28</td></tr><tr><td><eq>CR_{total}</eq>[%]</td><td>0.33</td><td>1.30</td><td>2.10</td><td>5.15</td><td>1.26</td><td>3.87</td></tr><tr><td>CD [cm]</td><td>2.51</td><td>2.24</td><td>2.44</td><td>4.05</td><td>2.47</td><td>3.75</td></tr><tr><td>AS [m/s]</td><td>0.67</td><td>0.07</td><td>0.65</td><td>0.70</td><td>0.66</td><td>0.72</td></tr><tr><td>CS</td><td>0.15</td><td>-1.21</td><td>-0.61</td><td>-2.38</td><td>-0.25</td><td>-1.69</td></tr></table>

Overall, our model $M _ { 0 }$ outperforms the other five models. It achieves the majority of the best values across the three performance metrics and the composite score. Moreover, it secures the highest composite scores in the three handcrafted scenarios, i.e., the Intersection, On-Ramp, and Roundabout Scenarios. Although it does not achieve the highest composite score in the CPM Scenario, its performance remains close to the model with the best composite score, scoring 0.24 versus 0.28 by model $M _ { 4 } )$ ). Owing to the omission of one of the proposed observation-design strategies, other five models underperform in some performance metrics: 

• Model $M _ { 1 }$ , which uses a bird-eye view instead of an ego view, underperforms in almost all performance metrics, likely owing to low learning efficiency during training. 

• Model $M _ { 2 } ,$ , which observes surrounding agents’ poses and geometric dimensions instead of their vertices, notably increases the agent-agent collision rate. Interestingly, it lowers the agent-lane collision rate, presumably because the ineffective observation of surrounding agents leads to an attention shift from surrounding agents to lanes. 

• Model $M _ { 3 }$ , which does not observe the distances to surrounding agents, suffers from the highest agent-agent collision rate in most scenarios. This highlights the critical role of distance observation in learning risk awareness to prevent collisions with other agents. 

• Model $M _ { 4 }$ , which observes the sampled points from the boundaries rather than the distances to them, excels in avoiding agent-lane collisions. However, it leads to a high agent-agent collision rate, possibly because it causes agents to overly focus on lane boundaries at the expense of neglecting their surrounding agents. 

• Model $M _ { 5 } { ^ \bf { \cdot } s }$ omission of observing distances to center lines results in poor lane-following performance compared to model $M _ { 0 } .$ It increases the average speed in the second, third, and fourth scenarios, primarily by compromising safety. 

In summary, our experiments underline the effectiveness of the proposed observation-design strategy in enhancing sample efficiency and generalization. 

# C. Limitations of Our Study

The composite score calculated through Eq. (2) favors conservative models. The most conservative model, which lets all agents stay stationary in a scenario, would get a composite score of zero, since all three performance metrics would be zero. Consequently, this score might misleadingly indicate better performance than other models, who may get negative scores if the scenario is challenging enough. 

# V. CONCLUSIONS

In this paper, we presented our open-source SigmaRL, a sample-efficient and generalizable multi-agent RL framework for motion planning of CAVs. We formulated the motion planning problem as a partially observable Markov game and explored the under-explored area of how observation design affects sample efficiency and generalization. As outcomes, we proposed five strategies for designing information-dense, structured observations that enhanced both the sample efficiency and the generalization of RL agents. These strategies focused on extracting general features applicable across various traffic scenarios. In our numerical experiments, we required only one million samples and less than one hour on a single CPU to train our agents, suggesting outstanding sample efficiency. Despite being only trained on an intersection, they demonstrated outstanding zero-shot generalization to completely unseen scenarios, including a new intersection, an on-ramp, and a roundabout. Our results suggested that our observation-design strategies may be a viable approach toward achieving sample efficient and generalizable MARL for motion planning of CAVs. 

Future work will include comparing our proposed observation design with image-based observations regarding sample efficiency and generalization. 

# REFERENCES



[1] S. Aradi, “Survey of deep reinforcement learning for motion planning of autonomous vehicles,” IEEE Transactions on Intelligent Transportation Systems, vol. 23, no. 2, pp. 740–759, 2022. 





[2] J. Farebrother, M. C. Machado, and M. Bowling, “Generalization and regularization in DQN,” arXiv preprint arXiv:1810.00123, 2020. 





[3] S. Feng, H. Sun, X. Yan, H. Zhu, Z. Zou, S. Shen, and H. X. Liu, “Dense reinforcement learning for safety validation of autonomous vehicles,” Nature, vol. 615, no. 7953, pp. 620–627, 2023. 





[4] Y. Duan, J. Schulman, X. Chen, P. L. Bartlett, I. Sutskever, and P. Abbeel, “RL2: Fast reinforcement learning via slow reinforcement learning,” arXiv preprint arXiv:1611.02779, 2016. 





[5] Y. Yu, “Towards sample efficient reinforcement learning,” Proceedings of the Twenty-Seventh International Joint Conference on Artificial Intelligence (IJCAI-18), 2018. 





[6] L.-J. Lin, “Self-improving reactive agents based on reinforcement learning, planning and teaching,” Machine Learning, vol. 8, no. 3, pp. 293–321, 1992. 





[7] S. Zhang and R. S. Sutton, “A deeper look at experience replay,” arXiv preprint arXiv:1712.01275, 2018. 





[8] T. Schaul, J. Quan, I. Antonoglou, and D. Silver, “Prioritized experience replay,” in International Conference on Learning Representations (ICLR), 2016. 





[9] E. Kaufmann, L. Bauersfeld, A. Loquercio, M. Muller, V. Koltun, and ¨ D. Scaramuzza, “Champion-level drone racing using deep reinforcement learning,” Nature, vol. 620, no. 7976, pp. 982–987, 2023. 





[10] Y. Song, A. Romero, M. Mueller, V. Koltun, and D. Scaramuzza, “Reaching the limit in autonomous racing: Optimal control versus reinforcement learning,” Science Robotics, vol. 8, no. 82, p. eadg1462, 2023. 





[11] J. Buckman, D. Hafner, G. Tucker, E. Brevdo, and H. Lee, “Sampleefficient reinforcement learning with stochastic ensemble value expansion,” in Advances in Neural Information Processing Systems, vol. 31. Curran Associates, Inc., 2018. 





[12] S. S. Du, S. M. Kakade, R. Wang, and L. F. Yang, “Is a good representation sufficient for sample efficient reinforcement learning?” in International Conference on Learning Representations, 2020. 





[13] C. Packer, K. Gao, J. Kos, P. Krahenb ¨ uhl, V. Koltun, and D. Song, “As- ¨ sessing generalization in deep reinforcement learning,” arXiv preprint arXiv:1810.12282, 2019. 





[14] V. Mnih, A. P. Badia, M. Mirza, A. Graves, T. Lillicrap, T. Harley, D. Silver, and K. Kavukcuoglu, “Asynchronous methods for deep reinforcement learning,” in Proceedings of The 33rd International Conference on Machine Learning. PMLR, 2016, pp. 1928–1937. 





[15] J. Schulman, F. Wolski, P. Dhariwal, A. Radford, and O. Klimov, “Proximal policy optimization algorithms,” arXiv preprint arXiv:1707.06347, 2017. 





[16] A. Rajeswaran, S. Ghotra, B. Ravindran, and S. Levine, “EPOpt: Learning robust neural network policies using model ensembles,” in International Conference on Learning Representations (ICLR), 2017. 





[17] K. Cobbe, O. Klimov, C. Hesse, T. Kim, and J. Schulman, “Quantifying generalization in reinforcement learning,” in Proceedings of the 36th International Conference on Machine Learning. PMLR, 2019, pp. 1282–1289. 





[18] Q. Li, Z. Peng, L. Feng, Q. Zhang, Z. Xue, and B. Zhou, “MetaDrive: Composing diverse driving scenarios for generalizable reinforcement learning,” IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 45, no. 3, pp. 3461–3475, 2023. 





[19] M. Igl, G. Farquhar, J. Luketina, W. Boehmer, and S. Whiteson, “Transient non-stationarity and generalisation in deep reinforcement learning,” in International Conference on Learning Representations, 2020. 





[20] R. Kirk, A. Zhang, E. Grefenstette, and T. Rocktaschel, “A survey of ¨ zero-shot generalisation in deep reinforcement learning,” Journal of Artificial Intelligence Research, vol. 76, pp. 201–264, 2023. 





[21] L. S. Shapley, “Stochastic games,” Proceedings of the National Academy of Sciences, vol. 39, no. 10, pp. 1095–1100, 1953. 





[22] F. A. Oliehoek and C. Amato, A Concise Introduction to Decentralized POMDPs. Cham: Springer International Publishing, 2016. 





[23] E. A. Hansen, D. S. Bernstein, and S. Zilberstein, “Dynamic programming for partially observable stochastic games,” American Association for Artificial Intelligence, pp. 709–715, 2024. 





[24] D. S. Bernstein, R. Givan, N. Immerman, and S. Zilberstein, “The complexity of decentralized control of Markov decision processes,” Mathematics of Operations Research, vol. 27, no. 4, pp. 819–840, 2002. 





[25] R. Lowe, YI. WU, A. Tamar, J. Harb, O. Pieter Abbeel, and I. Mordatch, “Multi-agent actor-critic for mixed cooperative-competitive environments,” in Advances in Neural Information Processing Systems, vol. 30. Curran Associates, Inc., 2017. 





[26] K. Zhang, Z. Yang, and T. Bas¸ar, “Multi-agent reinforcement learning: A selective overview of theories and algorithms,” in Handbook of Reinforcement Learning and Control. Cham: Springer International Publishing, 2021, pp. 321–384. 





[27] M. Bettini, R. Kortvelesy, J. Blumenkamp, and A. Prorok, “VMAS: A vectorized multi-agent simulator for collective robot learning,” in Distributed Autonomous Robotic Systems. Cham: Springer Nature Switzerland, 2024, pp. 42–56. 





[28] M. Kloock, P. Scheffe, J. Maczijewski, A. Kampmann, A. Mokhtarian, S. Kowalewski, and B. Alrifaee, “Cyber-Physical Mobility Lab: An open-source platform for networked and autonomous vehicles,” in 2021 European Control Conference (ECC), 2021, pp. 1937–1944. 





[29] R. Rajamani, Vehicle Dynamics and Control, ser. Mechanical Engineering Series. New York: Springer Science, 2006. 





[30] D. Li, D. Zhao, Q. Zhang, and Y. Chen, “Reinforcement learning and deep learning based lateral control for autonomous driving [application notes],” IEEE Computational Intelligence Magazine, vol. 14, no. 2, pp. 83–98, 2019. 





[31] M. Bansal, A. Krizhevsky, and A. Ogale, “Chauffeurnet: Learning to drive by imitating the best and synthesizing the worst,” arXiv preprint arXiv:1812.03079, 2018. 





[32] R. S. Sutton and A. G. Barto, Reinforcement Learning: An Introduction, second edition ed., ser. Adaptive Computation and Machine Learning Series. Cambridge, Massachusetts: The MIT Press, 2018. 





[33] P. Bender, J. Ziegler, and C. Stiller, “Lanelets: Efficient map representation for autonomous driving,” in 2014 IEEE Intelligent Vehicles Symposium Proceedings, 2014, pp. 420–425. 





[34] F. Poggenhans, J.-H. Pauls, J. Janosovits, S. Orf, M. Naumann, F. Kuhnt, and M. Mayr, “Lanelet2: A high-definition map framework for the future of automated driving,” in 2018 21st International Conference on Intelligent Transportation Systems (ITSC), 2018, pp. 1672–1679. 





[35] M. Haklay and P. Weber, “OpenStreetMap: User-generated street maps,” IEEE Pervasive Computing, vol. 7, no. 4, pp. 12–18, 2008. 

