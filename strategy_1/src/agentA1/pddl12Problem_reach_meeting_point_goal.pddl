(define (problem problem-agentA1)
	(:domain agentA1)
	(:init 
		( = (at_meeting_point) 0.000000 )
		( = (costs) 0)
	)
	(:goal (and ( >= (at_meeting_point) 1.000000)))
	(:metric minimize (costs))
)
