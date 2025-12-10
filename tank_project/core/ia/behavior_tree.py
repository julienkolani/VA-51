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
from .decisions import (
    is_enemy_too_close,
    has_line_of_sight,
    is_optimal_firing_range,
    should_retreat,
    find_nearest_cover,
    calculate_flank_position
)


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


# --- Action Functions ---

def action_retreat_to_cover(context):
    """
    Action: Find cover and set as target.
    
    Modifies context['ai_output'] with retreat target.
    """
    cover_pos = find_nearest_cover(context)
    
    if cover_pos is not None:
        context['ai_output']['target_position'] = cover_pos
        context['ai_output']['state'] = 'RETREAT'
        context['ai_output']['fire_request'] = False
        print("[BT] Action: RETREAT to cover at ({:.2f}, {:.2f})".format(
            cover_pos[0], cover_pos[1]))
        return NodeStatus.SUCCESS
    else:
        print("[BT] Action: RETREAT failed - no cover found")
        return NodeStatus.FAILURE


def action_aim_and_fire(context):
    """
    Action: Aim at enemy and request fire.
    
    Modifies context['ai_output'] with fire request.
    """
    human_pose = context.get('human_pose')
    
    if human_pose is None:
        return NodeStatus.FAILURE
    
    context['ai_output']['target_position'] = None  # Stay in place
    context['ai_output']['target_orientation'] = human_pose[:2]  # Aim at enemy
    context['ai_output']['fire_request'] = True
    context['ai_output']['state'] = 'ATTACK'
    
    print("[BT] Action: AIM AND FIRE at enemy")
    return NodeStatus.SUCCESS


def action_find_flank_position(context):
    """
    Action: Calculate flanking position.
    """
    flank_pos = calculate_flank_position(context)
    
    if flank_pos is not None:
        context['ai_output']['target_position'] = flank_pos
        context['ai_output']['state'] = 'FLANK'
        context['ai_output']['fire_request'] = False
        print("[BT] Action: FLANK to ({:.2f}, {:.2f})".format(
            flank_pos[0], flank_pos[1]))
        return NodeStatus.SUCCESS
    else:
        return NodeStatus.FAILURE


def action_move_to_flank(context):
    """
    Action: Move towards flanking position.
    """
    # This is handled by trajectory follower, just confirm we have a target
    target = context['ai_output'].get('target_position')
    if target is not None:
        print("[BT] Action: Moving to flank position")
        return NodeStatus.RUNNING
    return NodeStatus.FAILURE


def action_hunt_enemy(context):
    """
    Action: Move towards enemy's last known position.
    """
    human_pose = context.get('human_pose')
    
    if human_pose is not None:
        context['ai_output']['target_position'] = human_pose[:2]
        context['ai_output']['state'] = 'HUNT'
        context['ai_output']['fire_request'] = False
        print("[BT] Action: HUNT - moving to enemy position")
        return NodeStatus.SUCCESS
    
    return NodeStatus.FAILURE


def build_ai_behavior_tree():
    """
    Construct the main AI behavior tree.
    
    Structure:
    
    Selector (Root)
      +-- Sequence (SURVIVAL)
      |   +-- Condition: "enemy too close?"
      |   +-- Action: "retreat to cover"
      |
      +-- Sequence (ATTACK)
      |   +-- Condition: "have line of sight?"
      |   +-- Condition: "in optimal range?"
      |   +-- Action: "aim and request fire"
      |
      +-- Sequence (FLANK)
          +-- Action: "find flanking position"
          +-- Action: "move to flank"
    
    Returns:
        BTNode: root of the behavior tree
    """
    # SURVIVAL branch: retreat if enemy too close
    survival_sequence = Sequence("SURVIVAL", [
        Condition("enemy_too_close", is_enemy_too_close),
        Action("retreat_to_cover", action_retreat_to_cover)
    ])
    
    # ATTACK branch: fire if we have clear shot in optimal range
    attack_sequence = Sequence("ATTACK", [
        Condition("has_line_of_sight", has_line_of_sight),
        Condition("in_optimal_range", is_optimal_firing_range),
        Action("aim_and_fire", action_aim_and_fire)
    ])
    
    # FLANK branch: try to get into better position
    flank_sequence = Sequence("FLANK", [
        Action("find_flank_position", action_find_flank_position),
        Action("move_to_flank", action_move_to_flank)
    ])
    
    # HUNT branch: fallback - just chase enemy
    hunt_action = Action("hunt_enemy", action_hunt_enemy)
    
    # Root selector: try survival first, then attack, then flank, then hunt
    root = Selector("AI_ROOT", [
        survival_sequence,
        attack_sequence,
        flank_sequence,
        hunt_action
    ])
    
    print("[BT] Behavior tree constructed")
    return root


class BehaviorTreeExecutor:
    """
    Executor that runs the behavior tree each tick.
    """
    
    def __init__(self):
        self.tree = build_ai_behavior_tree()
    
    def execute(self, context):
        """
        Execute behavior tree with given context.
        
        Args:
            context: World state dict. Must contain:
                - ai_pose: (x, y, theta)
                - human_pose: (x, y, theta)
                - occupancy_grid: grid object
                
        Returns:
            dict: AI output decisions
        """
        # Initialize output structure
        context['ai_output'] = {
            'target_position': None,
            'target_orientation': None,
            'fire_request': False,
            'state': 'IDLE',
            'has_los': False
        }
        
        # Check LOS for output
        context['ai_output']['has_los'] = has_line_of_sight(context)
        
        # Execute tree
        status = self.tree.tick(context)
        
        print("[BT] Tree execution: {}".format(status.value))
        
        return context['ai_output']
