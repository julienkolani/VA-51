"""
Behavior Tree - AI Decision Making Framework

Implements a composable behavior tree for AI decision making:
- Selector nodes (try children until one succeeds)
- Sequence nodes (execute all children in order)
- Condition nodes (check world state)
- Action nodes (output decisions)

The AI does NOT modify game state directly.
It only returns INTENTIONS: (target_position, fire_request).

Logs: [BT] Node X succeeded/failed
"""

from enum import Enum
from abc import ABC, abstractmethod


class NodeStatus(Enum):
    """Behavior tree node execution status."""
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


class BTNode(ABC):
    """
    Base class for all behavior tree nodes.
    
    All nodes implement tick() which returns a NodeStatus.
    """
    
    def __init__(self, name):
        self.name = name
    
    @abstractmethod
    def tick(self, context):
        """
        Execute this node with given context.
        
        Args:
            context: dict with world state, robot poses, etc.
            
        Returns:
            NodeStatus
        """
        pass


class Selector(BTNode):
    """
    Selector node: tries children until one succeeds.
    
    Returns SUCCESS if any child succeeds.
    Returns FAILURE if all children fail.
    """
    
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
    
    def tick(self, context):
        for child in self.children:
            status = child.tick(context)
            if status == NodeStatus.SUCCESS:
                return NodeStatus.SUCCESS
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
        return NodeStatus.FAILURE


class Sequence(BTNode):
    """
    Sequence node: executes children in order.
    
    Returns SUCCESS if all children succeed.
    Returns FAILURE if any child fails.
    """
    
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
    
    def tick(self, context):
        for child in self.children:
            status = child.tick(context)
            if status == NodeStatus.FAILURE:
                return NodeStatus.FAILURE
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
        return NodeStatus.SUCCESS


class Condition(BTNode):
    """
    Condition node: checks a predicate function.
    
    Returns SUCCESS if predicate is True, FAILURE otherwise.
    """
    
    def __init__(self, name, predicate_fn):
        super().__init__(name)
        self.predicate_fn = predicate_fn
    
    def tick(self, context):
        if self.predicate_fn(context):
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class Action(BTNode):
    """
    Action node: executes an action function.
    
    The action function modifies context['ai_output'] with decisions.
    """
    
    def __init__(self, name, action_fn):
        super().__init__(name)
        self.action_fn = action_fn
    
    def tick(self, context):
        return self.action_fn(context)


def build_ai_behavior_tree():
    """
    Construct the main AI behavior tree.
    
    Structure:
    
    Selector (Root)
      ├─ Sequence (SURVIVAL)
      │   ├─ Condition: "enemy too close?"
      │   └─ Action: "retreat to cover"
      │
      ├─ Sequence (ATTACK)
      │   ├─ Condition: "have line of sight?"
      │   ├─ Condition: "in optimal range?"
      │   └─ Action: "aim and request fire"
      │
      └─ Sequence (FLANK)
          ├─ Action: "find flanking position"
          └─ Action: "move to flank"
    
    Returns:
        BTNode: root of the behavior tree
    """
    pass
