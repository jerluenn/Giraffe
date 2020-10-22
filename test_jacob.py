from casadi import *

x1 = MX.sym('x1')
x2 = MX.sym('x2')
x = [x1, x2]
u = MX.sym('u')

x1dot = (1 - x2**2)*x1 - x2 + u
x2dot = x1

f = Function('f', [x1, x2, u], [x1dot, x2dot])

T = 10
N = 20

# dae = DaeBuilder()

# x1 = dae.add_x('x1')
# x2 = dae.add_x('x2')
# u = dae.add_u('u')
# x1dot = dae.add_ode('x1dot', x1dot)
# x2dot = dae.add_ode('x2dot', x2dot)

# dae.set_start('x1', 0)
# dae.set_start('x2', 0)
# dae.set_start('u', 0)

# d = dae.create('d', ['x', 'u'], ['ode'])

d = {'x': [x1,x2], 'p': u, 'ode': [x1dot, x2dot]}

I = integrator('I', 'rk', d)




