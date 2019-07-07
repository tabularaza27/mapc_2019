(define (domain agentA1)
(:requirements :strips :adl :equality :negation :conditional-effects :fluents)
(:predicates
    (dispenser_visible)
    (all_positions_attached)
    (next_to_block)
    (attached_to_block)
    (assigned_task_list_empty)
    (at_the_dispenser)
    (at_meeting_point)
    (can_submit))
(:functions
    (costs)
    (at_meeting_point))
(:action exploration_move
:parameters ()
:effect (and (increase (costs) 1.0) (dispenser_visible) (assigned_task_list_empty))
)
(:action move_to_dispenser
:parameters ()
:precondition (and (not (assigned_task_list_empty)) (not (attached_to_block)) (not (at_the_dispenser)))
:effect (and (increase (costs) 1.0) (at_the_dispenser))
)
(:action dispense
:parameters ()
:precondition (and (not (assigned_task_list_empty)) (not (attached_to_block)) (at_the_dispenser) (not (next_to_block)))
:effect (and (increase (costs) 1.0) (next_to_block))
)
(:action attach
:parameters ()
:precondition (and (not (assigned_task_list_empty)) (not (attached_to_block)) (next_to_block) (not (all_positions_attached)))
:effect (and (increase (costs) 1.0) (attached_to_block))
)
(:action reach_meeting_point
:parameters ()
:precondition (and (not (assigned_task_list_empty)) (attached_to_block) (not (can_submit)) (not (at_meeting_point)))
:effect (and (increase (costs) 1.0) (at_meeting_point))
)
)