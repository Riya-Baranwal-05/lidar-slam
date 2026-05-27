    # 3. update with each landmark
    for landmark in landmarks:
        z = measure_landmark(true_pose, landmark)
        z_expected = expected_measurement(mu, landmark)
        innovation = z - z_expected
        H = compute_H(mu, landmark)
        mu, sigma = update(mu, sigma, innovation, H, R)