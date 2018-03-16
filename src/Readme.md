# PID Controller Project

From lesson 14 "Parameter Optimization":
Then substituting the parameter values suggested for lesson 17 "PID Implementation."
Using the manual tuning methodology outlined here: https://discussions.udacity.com/t/how-to-tune-parameters/303845/4

```python
# Make this tolerance bigger if you are timing out!
def twiddle(tol=0.01): 
    p = [0.2, 3.0, 0.004]
    dp = [0.1, 1.5, 0.002]
    #p = [0.2, 3.0, 0.0]
    #dp = [0.1, 1.5, 0.0]
    #p = [0, 0, 0]
    #dp = [1.0, 1.0, 1.0]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        #print("Iteration {}, best error = {}".format(it, best_err))
        #print(p)
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p, best_err
```
`Final twiddle error = 3.0519334332727137e-06
[2.793219409694487, 8.877885854779068, 0.03125677119852825]`
![Final twiddle results vs. lesson output](./twiddle.png)