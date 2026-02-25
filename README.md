# ViPlan--Here's-your-PDDL-Problem-File-On-Using-VLMs-for-Generating-Symbolic-PDDL-Problem-Files
This is the implementation of our paper accepted by ICRA 2025

![Untitleddesign4-ezgif com-video-to-gif-converter (1)](https://github.com/user-attachments/assets/6577140f-ad34-4483-a5cf-1fc404c9f46d)

Authors: Victor Aregbede, Paolo Forte, Himanshu Gupta, Henrik Andreasson, Uwe Köckemann, Achim J. Lilienthal

# Abstract:
Large Language Models (LLMs) excel at generating contextually relevant text but lack logical reasoning abilities. They rely on statistical patterns rather than logical inference, making them unreliable for structured decision-making. Integrating LLMs with task planning can address this limitation by combining their natural language understanding with the precise, goal-oriented reasoning of planners. This paper introduces ViPlan, a hybrid system that leverages Vision Language Models (VLMs) to extract high-level semantic information from visual and textual inputs while integrating classical planners for logical reasoning. ViPlan utilizes VLMs to generate syntactically correct and semantically meaningful PDDL problem files from images and natural language instructions, which are then processed by a task planner to generate an executable plan. The entire process is embedded within a behavior tree framework, enhancing efficiency, reactivity, replanning, modularity, and flexibility. The generation and planning capabilities of ViPlan are empirically evaluated with simulated and real-world experiments
