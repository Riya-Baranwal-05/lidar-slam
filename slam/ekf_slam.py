import numpy as np

def predict(mu, sigma, u, Q):
    """
    Inputs: 
        mu: current state estimate 
        sigma = current covariance matrix
        u = control input (distance travelled,change in heading)
        Q = Process noise matrix Q

    Returns: 
        new_mu: Next estimated state using motion model and control input
        new_sigma: new predicted covariance matrix using Process Jacobian F
        and Process noise Q
    How it works:
        The prediction step uses robot's motion model and control input to estimate
        where the robot should be next. It also takes the uncertainity forward with covariance 
        matrix

    """
    d,delta_theta = u
    x,y,theta = mu
    x_new = x + d*np.cos(theta)
    y_new = y + d*np.sin(theta)
    theta_new = theta + delta_theta

    new_mu = np.array([x_new,y_new,theta_new])
    F =np.array([[1, 0, -d*np.sin(theta)],
     [0, 1,  d*np.cos(theta)],
     [0, 0,  1       ]])
    
    new_sigma = F@sigma@F.T + Q

    return new_mu,new_sigma


def update(mu, sigma, z, H, R):
    """
    Inputs:
        mu: Predicted state from the prediction step
        sigma: new predicted covariance
        z: actual measurement of the sensor
        H: Measurement Jacobian Matrix
        R: Measurement Noise Covariance

    Output:
        new_mu = updated state estimate after correction
        new_sigma = updated covariance (smaller then prediction step)
    How it works:
        This step corrects the predicted robot state using new sensor measurement.
        It compares what sensor actually observed with what was expected, then corrects 
        both state estimate and covariance.
    """
   

    innovation = z - H@mu
    K = sigma@H.T@np.linalg.inv(H@sigma@H.T+R)
    new_mu = mu + K@innovation
    new_sigma = (np.eye(3)-K@H)@sigma