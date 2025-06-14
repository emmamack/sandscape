def parse_grbl_status(status_string):
    """
    Parses a GRBL status message string into a dictionary.
    It ignores anything not within the last <> in the string.

    Args:
        status_string (str): The GRBL status message, e.g.,
                             "Extra text here <Idle|MPos:0.000,0.000,0.000> Some other stuff"

    Returns:
        dict: A dictionary containing the parsed status information.
              Returns an empty dictionary if the string format is invalid
              or the last angle bracket pair is not found/malformed.
    """
    parsed_data = {}

    # Find the last occurrence of '<' and '>'
    last_open_bracket_idx = status_string.rfind('<')
    last_close_bracket_idx = status_string.rfind('>')

    # Validate that brackets are found and in correct order
    if last_open_bracket_idx == -1 or last_close_bracket_idx == -1 or \
       last_open_bracket_idx >= last_close_bracket_idx:
        print(f"Error: Invalid or missing GRBL status message format within '<>'.")
        return {}

    # Extract the content strictly within the last <>
    # The +1 on last_close_bracket_idx is to include the '>' character in the slice
    trimmed_status_string = status_string[last_open_bracket_idx : last_close_bracket_idx + 1]

    # Now, proceed with the original parsing logic on the trimmed string
    content = trimmed_status_string[1:-1]  # Remove leading '<' and trailing '>'
    components = content.split('|')

    if not components:
        print("Error: No components found in status message after trimming.")
        return {}

    # The first component is always the state
    parsed_data['State'] = components[0]

    # Process remaining components (field:value pairs)
    for component in components[1:]:
        if ':' in component:
            field, value = component.split(':', 1) # Split only on the first colon

            # MPos and WPos are stored as lists (X, Y only)
            if field == 'MPos' or field == 'WPos':
                try:
                    coords = [float(c) for c in value.split(',')]
                    # Only record X and Y as a list
                    if len(coords) >= 2:
                        parsed_data[field] = [coords[0], coords[2]]
                    else:
                        parsed_data[field] = value # Fallback to raw string if not enough coords
                except ValueError:
                    parsed_data[field] = value # Store as raw string if parsing fails
            elif field == 'FS': # Feed rate and Spindle speed
                try:
                    fs_values = [float(s) for s in value.split(',')]
                    parsed_data['FeedRate'] = fs_values[0]
                    parsed_data['SpindleSpeed'] = fs_values[1]
                except ValueError:
                    parsed_data[field] = value
            elif field == 'Bf' or field == 'RX': # Buffer states
                 try:
                    # Buf can be single value or two (planner, rx_chars)
                    if ',' in value:
                        buf_values = [int(s) for s in value.split(',')]
                        parsed_data['PlannerBuffer'] = buf_values[0]
                        parsed_data['RxBuffer'] = buf_values[1]
                    else:
                        parsed_data[field] = int(value)
                 except ValueError:
                    parsed_data[field] = value
            elif field == 'Ov': # Override values
                try:
                    overrides = [int(s) for s in value.split(',')]
                    parsed_data['FeedOverride'] = overrides[0]
                    parsed_data['RapidOverride'] = overrides[1]
                    parsed_data['SpindleOverride'] = overrides[2]
                except ValueError:
                    parsed_data[field] = value
            elif field == 'Pn' or field == 'A': # Pin status or Accessory state
                # These fields contain flags (e.g., "XYZ" or "SFM")
                parsed_data[field] = value
            elif field == 'Ln': # Line number
                try:
                    parsed_data[field] = int(value)
                except ValueError:
                    parsed_data[field] = value
            else:
                # Attempt to convert to float or int, otherwise keep as string
                try:
                    if '.' in value:
                        parsed_data[field] = float(value)
                    else:
                        parsed_data[field] = int(value)
                except ValueError:
                    parsed_data[field] = value
        else:
            # Handle cases where a component might just be a state flag without a colon,
            # (e.g., in a malformed or custom status string)
            if component:
                 parsed_data[f'Flag_{component}'] = True

    return parsed_data



# ### Example Usage

# # Example 1: Standard Idle state with some leading/trailing text
# status_msg1 = "Hello GRBL <Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000|Buf:15,RX:0|FS:0,0|Ov:100,100,100> End of message"
# print(f"Parsing: {status_msg1}")
# parsed_dict1 = parse_grbl_status(status_msg1)
# for key, value in parsed_dict1.items():
#     print(f"  {key}: {value}")
# print("-" * 30)

# # Example 2: Run state with multiple bracket pairs, only the last one is parsed
# status_msg2 = "<Error> something went wrong <Run|MPos:10.500,20.123,-5.000|WPos:10.500,20.123,-5.000|Buf:10|Pn:XZ|A:S|FS:500,12000|Ov:100,100,100>"
# print(f"Parsing: {status_msg2}")
# parsed_dict2 = parse_grbl_status(status_msg2)
# for key, value in parsed_dict2.items():
#     print(f"  {key}: {value}")
# print("-" * 30)

# # Example 3: Alarm state with only X and Y positions from MPos/WPos
# status_msg3 = "System startup... <Alarm|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000|Buf:5|FS:0,0|Pn:Y>"
# print(f"Parsing: {status_msg3}")
# parsed_dict3 = parse_grbl_status(status_msg3)
# for key, value in parsed_dict3.items():
#     print(f"  {key}: {value}")
# print("-" * 30)

# # Example 4: Status with more axes (Z, A, B, C will be ignored for MPos/WPos lists)
# status_msg4 = "Log: <Idle|MPos:10.0,20.0,30.0,1.0,2.0,3.0|WPos:5.0,10.0,15.0,0.5,1.0,1.5|Buf:15>"
# print(f"Parsing: {status_msg4}")
# parsed_dict4 = parse_grbl_status(status_msg4)
# for key, value in parsed_dict4.items():
#     print(f"  {key}: {value}")
# print("-" * 30)

# # Example 5: Invalid format (no closing bracket)
# status_msg5 = "Oops <Idle|MPos:1,2,3"
# print(f"Parsing: {status_msg5}")
# parsed_dict5 = parse_grbl_status(status_msg5)
# for key, value in parsed_dict5.items():
#     print(f"  {key}: {value}")
# print("-" * 30)