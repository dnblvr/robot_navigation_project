
<!-- omit in toc -->
# Learnbox Notes

This is primarily for Nabin on how to run this python script so that he has an easier time installing everything.

<!-- markdownlint-disable MD033 -->

<!-- omit in toc -->
## Contents

- [1. Advice: how to set up a virtual environment (venv) for Python in VSCode](#1-advice-how-to-set-up-a-virtual-environment-venv-for-python-in-vscode)
  - [1.1. Create a virtual environment](#11-create-a-virtual-environment)
  - [1.2. Activate the virtual environment](#12-activate-the-virtual-environment)
  - [1.3. Deactivate the virtual environment](#13-deactivate-the-virtual-environment)
  - [1.4. Install packages in the venv](#14-install-packages-in-the-venv)
  - [1.5. Save requirements](#15-save-requirements)
  - [1.6. Install from requirements](#16-install-from-requirements)
- [2. Regular Kalman Filter](#2-regular-kalman-filter)
- [3. Extended Kalman Filter](#3-extended-kalman-filter)
- [4. EKF-SLAM](#4-ekf-slam)

<!-- omit in toc -->
## Notes to Organize

---

## 1. Advice: how to set up a virtual environment (venv) for Python in VSCode

GitHub Copilot: To work with virtual environments (venv) in Python:

first, clone this whole project onto a directory of your choosing. Then, find  the path to the `.\Learnbox\` directory where the `.venv` will live and change it to this using `cd` (change directory).

```powershell
cd .\path\to\Learnbox\
```

### 1.1. Create a virtual environment

```bash
python -m venv .venv
```

### 1.2. Activate the virtual environment

**Windows (PowerShell):**

```powershell
.venv\Scripts\Activate.ps1
```

**Windows (Command Prompt):**

```cmd
.venv\Scripts\activate.bat
```

### 1.3. Deactivate the virtual environment

```bash
deactivate
```

### 1.4. Install packages in the venv

```bash
pip install numpy scipy matplotlib
```

### 1.5. Save requirements

```bash
pip freeze > requirements.txt
```

```powershell
pip freeze | Out-File requirements.txt
```

### 1.6. Install from requirements

```bash
pip install -r requirements.txt
```

I recommend installing from the `requirements.txt` file and keep updating this file 

**In VS Code:**

- Use `Cmd + Shift + P` → "Python: Select Interpreter" to choose your venv's Python interpreter
- This ensures your Jupyter notebook uses the correct environment

**Summary:**
Virtual environments isolate your project dependencies from your system Python installation.

## 2. Regular Kalman Filter

This assumes a motion model $x_k = f(x_{k-1})$ and the sensor observation model $z_k = h(z_k)$ to be **linear**.

- [ ] [How a Kalman filter works, in pictures | Bzarg][]

<!-- **Sources**: -->

[How a Kalman filter works, in pictures | Bzarg]: https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/ "How a Kalman filter works, in pictures | Bzarg"

## 3. Extended Kalman Filter

This assumes a motion model $x_k = f(x_{k-1})$ and the sensor observation model $z_k = h(z_k)$ to be **nonlinear**. The derivation is slightly different.

**Resources**:

- [ ] [PythonRobotics/Localization/extended_kalman_filter/extended_kalman_filter.py at master · AtsushiSakai/PythonRobotics][]
- [ ] [Special Topics - The Kalman Filter - Michel van Biezen - YouTube][]
- [ ] [Kalman Filter from the Ground Up - Alex Becker][] (the book)
- [ ] [Extended Kalman Filter Tutorial - Gabriel A. Terejanu][] (the more complicated-seeming resource)

<!-- **Sources**: -->

[PythonRobotics/Localization/extended_kalman_filter/extended_kalman_filter.py at master · AtsushiSakai/PythonRobotics]: https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter.py "PythonRobotics/Localization/extended_kalman_filter/extended_kalman_filter.py at master · AtsushiSakai/PythonRobotics"

[Special Topics - The Kalman Filter - Michel van Biezen - YouTube]: https://www.youtube.com/watch?v=CaCcOwJPytQ&list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT&index=1 "Special Topics - The Kalman Filter \(1 of 55\) What is a Kalman Filter? - YouTube"

[Kalman Filter from the Ground Up - Alex Becker]: <resources/Kalman Filter from the Ground Up - Alex Becker (2023).pdf> "Kalman Filter from the Ground Up - Alex Becker (2023)"

[Extended Kalman Filter Tutorial - Gabriel A. Terejanu]: <resources/Extended Kalman Filter Tutorial - Gabriel A. Terejanu.pdf> "Extended Kalman Filter Tutorial - Gabriel A. Terejanu"

## 4. EKF-SLAM

...
