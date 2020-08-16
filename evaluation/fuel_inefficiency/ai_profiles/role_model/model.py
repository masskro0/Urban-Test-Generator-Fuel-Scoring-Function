#vehicle.ai_set_speed(13.8)
# Manual gear control
# vehicle.set_shift_mode('realistic_manual_auto_clutch')
# vehicle.ai_set_aggression(0.3)        # Blocks everything else
"""
for _ in range(2000):
    vehicle.control(throttle=0.2)
    print(sensors['electrics']['values']['gear'])
    print(sensors['electrics']['values']['rpm'])
    if sensors['electrics']['values']['rpm'] > 2800 and sensors['electrics']['values']['gear'] < 6:
        next_gear = int(sensors['electrics']['values']['gear']) + 1
        vehicle.control(gear=next_gear)
    elif sensors['electrics']['values']['gear'] > 1 and \
            sensors['electrics']['values']['rpm'] < 1500:
        next_gear = int(sensors['electrics']['values']['gear']) - 1
        vehicle.control(gear=next_gear)
    elif np.linalg.norm(vehicle.state["vel"]) > 20:
        vehicle.control(gear=2)
"""