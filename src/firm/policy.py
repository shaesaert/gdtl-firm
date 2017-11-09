from random import choice

class Policy:
	def __init__(self, pa_policy, planner):
		'''Policy: wrapper around a product automaton policy
			
			Example usage:

			p = Policy(pa_policy, planner)
			
			target, prob = p.get_target(belief_state)

			while prob > 0.95:
				# ...steer towards target...

				p.update_events(['p0', 'p2'])   			# Report APs p0 p2 true during steering
				target, prob = p.get_target(belief_state)   # update target
		'''

		self.pa_policy = pa_policy   							# mapping B x A -> B
		self.planner = planner     								# A
		self.spec_state = planner.automaton.init.keys()[0]  	# current state in A

		init_pa_state = self.planner.pa.init.keys()[0]
		self.prob = self.planner.pa.g.node[init_pa_state]['value']

	def update_events(self, events):
		'''Report events to Policy'''
		event_binary = self.planner.automaton.bitmap_of_props(events)

		# Update spec_state
		for next_state in self.planner.automaton.g[self.spec_state]:
			if event_binary in self.planner.automaton.g[self.spec_state][next_state]['input']:
				self.spec_state = next_state
				break

	def get_target(self, belief_state):
		'''Get next target for current belief_state and probability of specification satisfaction
		'''
		cur_state = (belief_state, self.spec_state)

		if cur_state in self.pa_policy.keys():
			target = self.pa_policy[cur_state]
			proba = self.planner.pa.g.node[cur_state]['value']
			return target, proba
		else:
			return None, 0.
