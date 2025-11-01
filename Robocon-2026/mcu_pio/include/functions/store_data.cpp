// takes raw byte array, matches the interface and parses the data
void store_data(std::vector<uint8_t> payload) {

    if (!payload.empty()) {

        // find id
        uint8_t id = payload[0];
    
        // parse the struct based on the id
        if (id == PWM) {    
            // function to parse the struct
            pwm = parse_struct<Pwm>(payload);
            debug_state(std::to_string(pwm.m1)+" "+std::to_string(pwm.m2)+" "+std::to_string(pwm.m3)+" "+std::to_string(pwm.m4));
        
        }
    }
}
