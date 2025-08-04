#!/usr/bin/env python3

CATEGORY_PRIORITY = ["gender", "glasses", "shirt_color", "face_shape", "skin_tone"]

def get_category_priority_index(category):
    try:
        return CATEGORY_PRIORITY.index(category)
    except ValueError:
        return len(CATEGORY_PRIORITY)

def sort_by_priority(options_dict):
    """Sort categories by predefined CATEGORY_PRIORITY order"""
    return sorted((options_dict or {}).items(), key=lambda x: get_category_priority_index(x[0]))

def format_sentence(name, category, value):
    """
    Returns a natural language sentence depending on category type.
    - gender, glasses -> "<Name> is <Value>"
    - shirt_color, face_shape, skin_tone -> "<Name> has <Value> ..."
    """
    category = category.lower()
    value = value.replace("_", " ").capitalize()

    if category == "gender" or category == "glasses":
        return f"{name} is {value}"
    elif category == "shirt_color":
        return f"{name} has {value} shirt"
    elif category == "face_shape":
        return f"{name} has {value} face shape"
    elif category == "skin_tone":
        return f"{name} has {value} skin tone"
    else:
        # Default fallback for unknown categories
        return f" has {value}"

def assign_unique(people, used_categories, source_key, assignment):
    """Assign one category per person from a given source (Matching, Gemini, Camera)"""
    people_sorted = sorted(
        [(n, p) for n, p in people if n not in assignment],
        key=lambda x: len((x[1].get("Comparison_result", {}).get(source_key) or {}))
    )
    for name, details in people_sorted:
        options = sort_by_priority(details.get("Comparison_result", {}).get(source_key))
        for cat, val in options:
            if cat not in used_categories:
                sentence = format_sentence(name, cat, val)
                assignment[name] = sentence
                used_categories.add(cat)
                break

def choose_unique_categories(data):
    """
    Tiered deterministic algorithm:
    1. Assign Matching_characteristics (fewest options first)
    2. Assign Gemini_characteristics for unassigned
    3. Assign Camera_characteristics for unassigned
    4. Fallback if none available
    """
    people = [(name, details) for name, details in data.items()]
    assignment = {}
    used_categories = set()

    # Phase 1: Matching first
    assign_unique(people, used_categories, "Matching_characteristics", assignment)
    # Phase 2: Gemini fallback
    assign_unique(people, used_categories, "Gemini_characteristics", assignment)
    # Phase 3: Camera fallback
    assign_unique(people, used_categories, "Camera_characteristics", assignment)

    # Append Gemini_location if available
    for name, details in data.items():
        location = details.get("Gemini_location")
        if location:
            assignment[name] += f", {location}"
            
    # Phase 4: Fallback for unassigned
    for name, _ in people:
        if name not in assignment:
            assignment[name] = f"{name} has no unique category available"
        
    return assignment
