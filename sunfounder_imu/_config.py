import json
import os
from typing import Iterator, Any

class Config():
    """ Config class

    Args:
        config_file (str): config file path
    """
    def __init__(self, config_file: str) -> None:
        self.config_file = config_file
        
        if not os.path.isfile(config_file):
            from pathlib import Path
            Path(config_file).parent.mkdir(parents=True, exist_ok=True)
            Path(config_file).touch()
            with open(config_file, 'w') as f:
                f.write('{}')
        with open(config_file, 'r') as f:
            content = f.read()
            if content == '':
                content = '{}'
            self._config = json.loads(content)

    def get(self, key: str, default: Any = None) -> Any:
        """ Get the value of the key

        Args:
            key (str): key name
            default (optional): default value if the key is not found. Defaults to None.

        Returns:
            Any: value of the key
        """
        return self._config.get(key, default)

    def set(self, key: str, value: Any, save: bool = True) -> None:
        """ Set the value of the key

        Args:
            key (str): key name
            value (Any): value of the key
        """
        self._config[key] = value
        if save:
            self.save()

    def save(self) -> None:
        """ Save the config to the file
        """
        content = "{\n"
        for key, value in self._config.items():
            content += f'  "{key}": {json.dumps(value)},\n'
        content = content[:-2] + "\n}"
        with open(self.config_file, 'w') as f:
            f.write(content)

    def delete(self, key: str) -> None:
        """ Delete the key

        Args:
            key (str): key name
        """
        if key in self._config:
            del self._config[key]
            with open(self.config_file, 'w') as f:
                json.dump(self._config, f, indent=4)

    def __getitem__(self, key: str) -> Any:
        """
        Get the value of the key

        Args:
            key (str): key name

        Returns:
            Any: value of the key
        """
        return self.get(key)

    def __setitem__(self, key: str, value: Any) -> None:
        """ Set the value of the key

        Args:
            key (str): key name
            value (Any): value of the key
        """
        self.set(key, value)

    def __delitem__(self, key: str) -> None:
        """ Delete the key

        Args:
            key (str): key name
        """
        self.delete(key)

    def __contains__(self, key: str) -> bool:
        """ Check if the key exists

        Args:
            key (str): key name

        Returns:
            bool: True if the key exists, False otherwise
        """
        return key in self._config

    def __iter__(self) -> Iterator[str]:
        """ Iterate over the keys

        Returns:
            Iterator[str]: iterator over the keys
        """
        return iter(self._config)

    def __len__(self) -> int:
        """ Get the number of keys

        Returns:
            int: number of keys
        """
        return len(self._config)

    def __str__(self) -> str:
        """ Get the string representation of the config

        Returns:
            str: string representation of the config
        """
        return json.dumps(self._config, indent=4)

    def __repr__(self) -> str:
        """ Get the string representation of the config

        Returns:
            str: string representation of the config
        """
        return f'Config({self.config_file})'
