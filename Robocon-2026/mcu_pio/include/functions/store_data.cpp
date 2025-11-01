// takes raw byte array, matches the interface and parses the data
void store_data(std::vector<uint8_t> payload) {

    if (!payload.empty()) {

        // find id
        uint8_t id = payload[0];
    
        // parse the struct based on the id
        if (id == PWM) {
            
            // function to parse the struct
            Pwm pwm = parse_struct<Pwm>(payload);             
        }

        if (id == BNOREADING) {
            
            // function to parse the struct
            BnoReading bnoreading = parse_struct<BnoReading>(payload); // count.a, count.b, count.c, count.d, based on your interface
            
        }

        if (id ==LIDAR) {
            
            // function to parse the struct
            Lidar lidar = parse_struct<Lidar>(payload); // count.a, count.b, count.c, count.d, based on your interface
            
        }

        if (id ==MOTORSPEEDS) {
            
            // function to parse the struct
            MotorSpeeds motorspeeds = parse_struct<MotorSpeeds>(payload); // count.a, count.b, count.c, count.d, based on your interface
            
        }
    }
}
