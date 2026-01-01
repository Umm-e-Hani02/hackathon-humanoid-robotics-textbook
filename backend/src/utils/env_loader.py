import os
from dotenv import load_dotenv

def load_env_variables(env_path=".env"):
    """
    Loads environment variables from a .env file.

    Args:
        env_path (str): The path to the .env file. Defaults to ".env".
    """
    load_dotenv(dotenv_path=env_path)

def get_env_variable(key: str, default: str = None) -> str:
    """
    Retrieves an environment variable.

    Args:
        key (str): The name of the environment variable.
        default (str): The default value to return if the variable is not found.

    Returns:
        str: The value of the environment variable.

    Raises:
        ValueError: If the environment variable is not found and no default is provided.
    """
    value = os.getenv(key)
    if value is None and default is None:
        raise ValueError(f"Environment variable '{key}' not set and no default provided.")
    return value if value is not None else default

if __name__ == "__main__":
    # Example usage:
    # Create a dummy .env file for testing
    with open(".env.test", "w") as f:
        f.write("TEST_VAR=test_value\n")
        f.write("ANOTHER_VAR=another_value\n")

    load_env_variables(env_path=".env.test")

    try:
        test_var = get_env_variable("TEST_VAR")
        print(f"TEST_VAR: {test_var}")

        another_var = get_env_variable("ANOTHER_VAR", "default_another")
        print(f"ANOTHER_VAR: {another_var}")

        missing_var = get_env_variable("MISSING_VAR", "default_missing")
        print(f"MISSING_VAR (with default): {missing_var}")

        # This will raise a ValueError
        # missing_var_no_default = get_env_variable("MISSING_VAR_NO_DEFAULT")
        # print(f"MISSING_VAR_NO_DEFAULT: {missing_var_no_default}")

    except ValueError as e:
        print(f"Error: {e}")

    # Clean up dummy .env file
    os.remove(".env.test")
